#!/usr/bin/env python3
"""
drone_controller_node.py

One instance of this node runs per physical/simulated drone, launched inside
that drone's MAVROS namespace (e.g. /drone1, /drone2). It wraps MAVROS
arm/set_mode/takeoff/land/setpoint calls behind two things the rest of
AeroRelay Hub actually talks to:

  - a periodic aerorelay_msgs/DroneState publication (fleet status)
  - an aerorelay_msgs/AssignMission service (fleet command)

The hub (mission_manager) never touches MAVROS directly. It only sees this
node's state and calls this node's service. That boundary is what lets us
swap in real hardware later without changing the hub at all.
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Float32, Float64, String
from sensor_msgs.msg import LaserScan, JointState
from mavros_msgs.msg import State as MavrosState
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

from aerorelay_msgs.msg import DroneState, DockPosition
from aerorelay_msgs.srv import AssignMission


SETPOINT_STREAM_DURATION_SEC = 8.0
SETPOINT_STREAM_RATE_HZ = 10
TAKEOFF_SETTLE_SEC = 15.0
LAND_SETTLE_SEC = 18.0
LAND_APPROACH_SETTLE_SEC = 8.0
MIN_BATTERY_FOR_MISSION = 20.0

# Obstacle avoidance tuning (locked design: 3m detect, 2m climb increments,
# 3-position servo sweep, retry every 2s gated by battery, RTL at 20%)
OBSTACLE_DETECT_DISTANCE_M = 1.5
OBSTACLE_CLIMB_INCREMENT_M = 2.0
OBSTACLE_LIDAR_MAX_RANGE_M = 5.0  # matches sensor's own <max> in the SDF;
                                   # readings beyond this are invalid, not "clear"
SERVO_FRONT_RAD = 0.0
SERVO_UP_RAD = 1.5708
SERVO_JOINT_NAME = 'lidar_servo_joint'
SERVO_POSITION_TOLERANCE_RAD = 0.1
SERVO_SETTLE_TIMEOUT_SEC = 3.0
OBSTACLE_RETRY_INTERVAL_SEC = 2.0
BATTERY_ABORT_FOR_OBSTACLE = 20.0
NAV_POSITION_TOLERANCE_M = 0.5
NAV_TIMEOUT_SEC = 60.0

# _ensure_disarmed tuning
DISARM_CONFIRM_TIMEOUT_SEC = 30.0
DISARM_POLL_INTERVAL_SEC = 0.5
DISARM_RESEND_INTERVAL_SEC = 5.0


class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller_node')

        self.declare_parameter('drone_id', 'drone1')
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value

        self._state_lock = threading.Lock()
        self.state = DroneState.STATE_IDLE
        self._state_since_ns = self.get_clock().now().nanoseconds
        self.current_mission_id = ''
        self._current_task_type = ''
        self.battery_percent = 100.0
        self.local_position = Point()
        self.healthy = True
        self.health_message = ''
        self._lidar_range = None
        self._servo_position_rad = None
        self._own_dock_position = None
        # Guards against the recursion _navigate_to_target -> _handle_obstacle
        # -> _navigate_to_target(climb) -> _handle_obstacle -> ... :
        # obstacle checks are skipped while already inside an obstacle
        # response, and again while performing RTL (RTL under critical
        # battery should not detour into a second avoidance sequence).
        self._handling_obstacle = False
        self._performing_rtl = False

        # Full MAVROS state (connected/armed/mode), guarded separately from
        # _state_lock since it's updated from a subscription callback and
        # read from the mission-execution thread at a different cadence.
        self._mavros_state_lock = threading.Lock()
        self.mavros_state = MavrosState()  # defaults: connected=False, armed=False

        cb_group = ReentrantCallbackGroup()

        qos = QoSProfile(depth=10,
                          reliability=QoSReliabilityPolicy.BEST_EFFORT,
                          history=QoSHistoryPolicy.KEEP_LAST)

        self.create_subscription(MavrosState, 'state', self._mavros_state_cb, qos)
        self.create_subscription(PoseStamped, 'local_position/pose',
                                  self._local_position_cb, qos)
        self.create_subscription(Float32, 'battery/percent', self._battery_cb, 10)
        self.create_subscription(LaserScan, 'obstacle_lidar', self._lidar_cb, 10)
        self.create_subscription(JointState, 'joint_states', self._servo_state_cb, 10)
        self.create_subscription(
            DockPosition, '/hub/dock_positions', self._dock_position_cb, 10)
        self.servo_cmd_pub = self.create_publisher(Float64, 'lidar_servo_cmd', 10)
        # Exposes the current mission leg's task_type ('PICKUP'/'DELIVER'/'RETURN')
        # so other nodes (precision_landing_node) can distinguish a hub-return LAND
        # from a pickup/delivery LAND without duplicating that information anywhere.
        self.task_type_pub = self.create_publisher(String, 'drone_controller/current_task_type', 10)

        self.arm_client = self.create_client(CommandBool, 'cmd/arming', callback_group=cb_group)
        self.mode_client = self.create_client(SetMode, 'set_mode', callback_group=cb_group)
        self.takeoff_client = self.create_client(CommandTOL, 'cmd/takeoff', callback_group=cb_group)
        self.land_client = self.create_client(CommandTOL, 'cmd/land', callback_group=cb_group)

        self.setpoint_pub = self.create_publisher(PoseStamped, 'setpoint_position/local', 10)

        self.state_pub = self.create_publisher(DroneState, 'drone_controller/state', 10)
        self.assign_mission_srv = self.create_service(
            AssignMission, 'drone_controller/assign_mission',
            self._assign_mission_cb, callback_group=cb_group)

        self.create_timer(0.5, self._publish_state)

        self.get_logger().info(f'[{self.drone_id}] drone_controller ready, state=IDLE')

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------
    def _mavros_state_cb(self, msg: MavrosState):
        with self._mavros_state_lock:
            self.mavros_state = msg

        if not msg.connected:
            self.healthy = False
            self.health_message = 'MAVROS link disconnected'
        else:
            if self.health_message == 'MAVROS link disconnected':
                self.healthy = True
                self.health_message = ''

    def _local_position_cb(self, msg: PoseStamped):
        self.local_position = msg.pose.position

    def _battery_cb(self, msg: Float32):
        self.battery_percent = msg.data

    def _lidar_cb(self, msg: LaserScan):
        """Reject inf/nan/zero/out-of-range readings -- only a genuinely
        valid measurement counts as data. Everything else leaves
        _lidar_range as whatever it was (stale-but-known) or None
        (never-yet-valid), and callers already treat None as
        'no valid reading' rather than 'clear'."""
        if not msg.ranges:
            return
        r = msg.ranges[0]
        if not math.isfinite(r):
            return
        if r <= 0.0 or r > OBSTACLE_LIDAR_MAX_RANGE_M:
            return
        self._lidar_range = r

    def _servo_state_cb(self, msg: JointState):
        if SERVO_JOINT_NAME in msg.name:
            idx = msg.name.index(SERVO_JOINT_NAME)
            if idx < len(msg.position):
                self._servo_position_rad = msg.position[idx]

    def _dock_position_cb(self, msg: DockPosition):
        if msg.drone_id == self.drone_id:
            self._own_dock_position = Point(x=msg.x, y=msg.y, z=msg.z)

    # ------------------------------------------------------------------
    # MAVROS state accessors (ground truth, not service ack)
    # ------------------------------------------------------------------
    def _is_connected(self):
        with self._mavros_state_lock:
            return self.mavros_state.connected

    def _is_armed(self):
        with self._mavros_state_lock:
            return self.mavros_state.armed

    # ------------------------------------------------------------------
    # State publication
    # ------------------------------------------------------------------
    def _publish_state(self):
        msg = DroneState()
        msg.drone_id = self.drone_id
        with self._state_lock:
            msg.state = self.state
            msg.current_mission_id = self.current_mission_id
            state_since_ns = self._state_since_ns
        msg.battery_percent = self.battery_percent
        msg.position = self.local_position
        msg.healthy = self.healthy and self._is_connected()
        msg.health_message = self.health_message
        # stamp reflects when THIS STATE was entered, not the publish cycle --
        # this is what lets mission_manager tell a fresh arrival apart from a
        # stale cached value left over from a previous mission leg.
        msg.stamp.sec = int(state_since_ns // 1_000_000_000)
        msg.stamp.nanosec = int(state_since_ns % 1_000_000_000)
        self.state_pub.publish(msg)

    def _set_state(self, new_state, mission_id=None):
        with self._state_lock:
            self.state = new_state
            self._state_since_ns = self.get_clock().now().nanoseconds
            if mission_id is not None:
                self.current_mission_id = mission_id
        self.get_logger().info(f'[{self.drone_id}] state -> {new_state}')

    # ------------------------------------------------------------------
    # AssignMission service
    # ------------------------------------------------------------------
    def _assign_mission_cb(self, request, response):
        with self._state_lock:
            busy = self.state not in (DroneState.STATE_IDLE, DroneState.STATE_READY,
                                       DroneState.STATE_WAITING, DroneState.STATE_CHARGING)

        if busy:
            response.accepted = False
            response.reason = f'BUSY (currently {self.state})'
            return response

        if self.battery_percent < MIN_BATTERY_FOR_MISSION:
            response.accepted = False
            response.reason = 'LOW_BATTERY'
            return response

        if not self._is_connected():
            response.accepted = False
            response.reason = 'NOT_CONNECTED'
            return response

        response.accepted = True
        response.reason = ''

        mission_thread = threading.Thread(
            target=self._execute_mission,
            args=(request.mission_id, request.target_location, request.task_type),
            daemon=True,
        )
        mission_thread.start()
        return response

    # ------------------------------------------------------------------
    # Mission execution (background thread)
    # ------------------------------------------------------------------
    def _execute_mission(self, mission_id, target_location, task_type):
        self.get_logger().info(
            f'[{self.drone_id}] starting mission {mission_id} ({task_type}) '
            f'-> ({target_location.x}, {target_location.y}, {target_location.z})')

        self._current_task_type = task_type
        self.task_type_pub.publish(String(data=task_type))

        self._set_state(DroneState.STATE_READY, mission_id=mission_id)

        if not self._call_set_mode('GUIDED'):
            self._fail_mission('Failed to set GUIDED mode')
            return

        if not self._call_arm(True):
            self._fail_mission('Failed to arm')
            return
        if not self._confirm_telemetry(lambda: self._is_armed(), 10.0):
            self._fail_mission('Arm command acked but /state.armed never became true')
            return

        self._set_state(DroneState.STATE_TAKEOFF)
        takeoff_alt = target_location.z if target_location.z > 0 else 5.0
        if not self._call_takeoff(takeoff_alt):
            self._fail_mission('Takeoff command rejected')
            return
        if not self._confirm_telemetry(
                lambda: abs(self.local_position.z - takeoff_alt) <= 0.5,
                TAKEOFF_SETTLE_SEC + 20.0):
            self._fail_mission(
                f'Takeoff acked but altitude never reached {takeoff_alt}m '
                f'(last z={self.local_position.z:.2f}m)')
            return

        self._set_state(DroneState.STATE_NAVIGATE)
        if not self._navigate_to_target(target_location, NAV_POSITION_TOLERANCE_M,
                                         NAV_TIMEOUT_SEC):
            self._rtl_to_own_dock()
            self._fail_mission(
                'Navigation did not reach target (timeout or unrecoverable '
                'obstacle/battery abort); RTL attempted')
            return
        self.get_logger().info(
            f'[{self.drone_id}] target reached, settling before landing')

        time.sleep(LAND_APPROACH_SETTLE_SEC)
        self._set_state(DroneState.STATE_LAND)
        if not self._call_land():
            self._fail_mission('Land command rejected')
            return
        time.sleep(LAND_SETTLE_SEC)

        if not self._ensure_disarmed():
            self._fail_mission('Could not confirm disarm after landing; '
                                'refusing to start next leg in an inconsistent state')
            return

        self._current_task_type = ''
        self.task_type_pub.publish(String(data=''))

        self._set_state(DroneState.STATE_WAITING)
        self.get_logger().info(f'[{self.drone_id}] mission {mission_id} leg complete, waiting')

    def _ensure_disarmed(self, timeout_sec=DISARM_CONFIRM_TIMEOUT_SEC):
        """
        Robust disarm confirmation using MAVROS telemetry as ground truth,
        not the CommandBool service ack.

        Why the old approach broke: ArduPilot auto-disarms shortly after
        landing (LAND_DISARMDELAY). If our explicit disarm command arrives
        after that auto-disarm already completed, ArduPilot correctly
        rejects it as redundant -- COMMAND_ACK comes back FAILED even
        though the vehicle genuinely IS disarmed. The bug was treating
        that ack as the source of truth: it kept retrying a command that
        had nothing left to do, and gave up even though the real goal
        (disarmed) had already been reached.

        Fix: check /state.armed directly. Only send a disarm command if
        still armed; then poll telemetry (not the ack) to confirm the
        transition, tolerating MAVROS's normal publish latency.
        """
        deadline = time.time() + timeout_sec

        if not self._is_armed():
            return True  # already disarmed (e.g. auto-disarmed on landing) -- nothing to do

        last_command_sent = 0.0
        while time.time() < deadline:
            if not self._is_armed():
                return True

            now = time.time()
            if now - last_command_sent >= DISARM_RESEND_INTERVAL_SEC:
                # Fire-and-forget: we don't trust the ack either way, we
                # only trust the subsequent telemetry read above.
                self._call_arm(False)
                last_command_sent = now

            time.sleep(DISARM_POLL_INTERVAL_SEC)

        # Final authoritative check in case armed flipped false right at
        # the deadline boundary.
        return not self._is_armed()

    def _confirm_telemetry(self, condition_fn, timeout_sec, poll_sec=0.3):
        """Small shared poll helper: block until condition_fn() is true or
        timeout_sec elapses. Used to verify real telemetry (armed, altitude)
        instead of trusting a MAVLink command ack."""
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if condition_fn():
                return True
            time.sleep(poll_sec)
        return condition_fn()

    def _fail_mission(self, reason):
        self._current_task_type = ''
        self.task_type_pub.publish(String(data=''))
        self.healthy = False
        self.health_message = reason
        self._set_state(DroneState.STATE_IDLE, mission_id='')
        self.get_logger().error(f'[{self.drone_id}] mission failed: {reason}')

    def _stream_setpoint(self, target_location, duration_sec):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = target_location.x
        msg.pose.position.y = target_location.y
        msg.pose.position.z = target_location.z if target_location.z > 0 else 5.0
        msg.pose.orientation.w = 1.0

        cycles = int(duration_sec * SETPOINT_STREAM_RATE_HZ)
        for _ in range(cycles):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.setpoint_pub.publish(msg)
            time.sleep(1.0 / SETPOINT_STREAM_RATE_HZ)

    def _distance_to(self, target_location):
        dx = self.local_position.x - target_location.x
        dy = self.local_position.y - target_location.y
        dz = self.local_position.z - target_location.z
        return (dx * dx + dy * dy + dz * dz) ** 0.5

    def _navigate_to_target(self, target_location, tolerance, timeout_sec):
        """
        Replaces the old fixed-duration _stream_setpoint call. Streams
        setpoints continuously while checking real distance-to-target each
        cycle, and checking the forward LiDAR for an obstacle -- but only
        when not already inside an obstacle response or an RTL (guards
        prevent the recursive nav->obstacle->nav->obstacle chain).

        Returns True ONLY if the drone actually entered tolerance before
        timeout. A timeout is a real failure, not silently treated as
        arrival -- this was a bug in the previous version of this patch.
        Returns False on timeout, or if _handle_obstacle had to abort for
        unrecoverable low battery.
        """
        deadline = time.time() + timeout_sec
        period = 1.0 / SETPOINT_STREAM_RATE_HZ

        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = target_location.x
        msg.pose.position.y = target_location.y
        msg.pose.position.z = target_location.z if target_location.z > 0 else 5.0
        msg.pose.orientation.w = 1.0

        while time.time() < deadline:
            if (not self._handling_obstacle and not self._performing_rtl and
                    self._lidar_range is not None and
                    self._lidar_range <= OBSTACLE_DETECT_DISTANCE_M):
                if not self._handle_obstacle():
                    return False

            msg.header.stamp = self.get_clock().now().to_msg()
            self.setpoint_pub.publish(msg)

            if self._distance_to(msg.pose.position) <= tolerance:
                return True

            time.sleep(period)

        # Timeout: explicitly check tolerance one last time rather than
        # assuming failure OR silently assuming success.
        return self._distance_to(msg.pose.position) <= tolerance

    def _set_servo(self, target_rad, timeout_sec=SERVO_SETTLE_TIMEOUT_SEC):
        """Command servo and confirm arrival via _servo_position_rad
        (populated by _servo_state_cb from real joint-state feedback) --
        not open-loop. Returns False on timeout; unconfirmed position is
        treated as 'not clear', never assumed safe."""
        cmd = Float64()
        cmd.data = target_rad
        self.servo_cmd_pub.publish(cmd)

        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if (self._servo_position_rad is not None and
                    abs(self._servo_position_rad - target_rad) <= SERVO_POSITION_TOLERANCE_RAD):
                return True
            time.sleep(0.1)
        return False

    def _handle_obstacle(self):
        """
        Front obstacle detected. Hold position -> rotate servo up
        (confirmed via real feedback) -> verify clearance above -> climb
        OBSTACLE_CLIMB_INCREMENT_M -> rotate servo back to front -> resume.
        Retries every OBSTACLE_RETRY_INTERVAL_SEC on any unconfirmed step,
        until battery drops to BATTERY_ABORT_FOR_OBSTACLE, at which point
        this returns False and the caller RTLs.

        self._handling_obstacle guards the climb's own _navigate_to_target
        call from re-triggering this same method recursively if the LiDAR
        still reads the obstacle mid-climb.
        """
        self._handling_obstacle = True
        try:
            self.get_logger().warn(
                f'[{self.drone_id}] obstacle at {self._lidar_range:.2f}m, holding')

            hold_msg = PoseStamped()
            hold_msg.header.frame_id = 'map'
            hold_msg.pose.orientation.w = 1.0

            while True:
                if self.battery_percent <= BATTERY_ABORT_FOR_OBSTACLE:
                    self.get_logger().error(
                        f'[{self.drone_id}] battery {self.battery_percent:.1f}% during '
                        f'obstacle retry -- aborting to RTL')
                    return False

                hold_msg.header.stamp = self.get_clock().now().to_msg()
                hold_msg.pose.position = self.local_position
                self.setpoint_pub.publish(hold_msg)

                if not self._set_servo(SERVO_UP_RAD):
                    time.sleep(OBSTACLE_RETRY_INTERVAL_SEC)
                    continue

                if self._lidar_range is None or self._lidar_range <= OBSTACLE_CLIMB_INCREMENT_M:
                    time.sleep(OBSTACLE_RETRY_INTERVAL_SEC)
                    continue

                climb_target = Point(x=self.local_position.x, y=self.local_position.y,
                                      z=self.local_position.z + OBSTACLE_CLIMB_INCREMENT_M)
                if not self._navigate_to_target(climb_target, NAV_POSITION_TOLERANCE_M, 20.0):
                    time.sleep(OBSTACLE_RETRY_INTERVAL_SEC)
                    continue

                if not self._set_servo(SERVO_FRONT_RAD):
                    time.sleep(OBSTACLE_RETRY_INTERVAL_SEC)
                    continue

                self.get_logger().info(f'[{self.drone_id}] obstacle cleared, resuming')
                return True
        finally:
            self._handling_obstacle = False

    def _rtl_to_own_dock(self):
        """self._performing_rtl guards against _navigate_to_target
        triggering a fresh _handle_obstacle (and thus a fresh potential
        RTL) while we are already in the middle of RTL under critical
        battery -- during RTL, obstacle checks are skipped entirely and
        we fly the direct path, accepting that risk deliberately rather
        than looping RTL-inside-RTL."""
        if self._own_dock_position is None:
            self.get_logger().error(
                f'[{self.drone_id}] RTL requested but no dock position known yet')
            return
        self._performing_rtl = True
        try:
            self.get_logger().warn(f'[{self.drone_id}] RTL to own dock (low battery)')
            self._navigate_to_target(self._own_dock_position, NAV_POSITION_TOLERANCE_M,
                                      NAV_TIMEOUT_SEC)
        finally:
            self._performing_rtl = False

    def _wait_for_future(self, future, timeout_sec=10.0):
        start = time.time()
        while not future.done():
            if time.time() - start > timeout_sec:
                return None
            time.sleep(0.1)
        return future.result()

    def _call_set_mode(self, mode):
        if not self.mode_client.wait_for_service(timeout_sec=5.0):
            return False
        req = SetMode.Request()
        req.custom_mode = mode
        result = self._wait_for_future(self.mode_client.call_async(req))
        return bool(result and result.mode_sent)

    def _call_arm(self, value):
        if not self.arm_client.wait_for_service(timeout_sec=5.0):
            return False
        req = CommandBool.Request()
        req.value = value
        result = self._wait_for_future(self.arm_client.call_async(req))
        return bool(result and result.success)

    def _call_takeoff(self, altitude):
        if not self.takeoff_client.wait_for_service(timeout_sec=5.0):
            return False
        req = CommandTOL.Request()
        req.altitude = altitude
        result = self._wait_for_future(self.takeoff_client.call_async(req))
        return bool(result and result.success)

    def _call_land(self):
        if not self.land_client.wait_for_service(timeout_sec=5.0):
            return False
        req = CommandTOL.Request()
        result = self._wait_for_future(self.land_client.call_async(req))
        return bool(result and result.success)


def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
