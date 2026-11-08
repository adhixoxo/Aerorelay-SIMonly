#!/usr/bin/env python3
"""
precision_landing_node.py

Precision landing, triggered ONLY when:
    drone_controller/current_task_type == 'RETURN'   (hub-bound leg;
                                                        pickup/delivery
                                                        legs use 'PICKUP'/
                                                        'DELIVER' and are
                                                        never triggered)
    AND mavros mode == 'LAND'

Does not do its own PID/velocity control. Once triggered, this node:
  - enables apriltag_detector_node (apriltag/enable = True)
  - rotates the shared LiDAR servo downward (reusing the existing
    lidar_servo_cmd topic already owned by drone_controller_node's
    obstacle-avoidance logic -- no conflict occurs because
    drone_controller only drives that servo during its NAVIGATE phase,
    never during LAND, so there is nothing to arbitrate)
  - relays each AprilTag detection as a MAVLink LANDING_TARGET message
    via MAVROS's landing_target plugin

ArduPilot (with PLND_ENABLED=1, PLND_TYPE=1 for companion-computer
input -- verify this is set in your SITL/vehicle params) owns all
horizontal alignment, descent rate, and final touchdown from here.
This node never publishes velocity or position setpoints and never
calls cmd/land itself -- drone_controller already commanded LAND before
this node's trigger condition can even become true.

On deactivation (mode leaves LAND, or task_type changes): disables the
detector and restores the servo to front, so obstacle avoidance is
ready again for the next mission leg.

KNOWN UNVERIFIED ASSUMPTION: LandingTarget.frame and which of
position vs angle_x/angle_y+distance MAVROS/ArduPilot actually expects
for this vehicle/version has not been empirically confirmed. This uses
frame=LOCAL_NED with the AprilTag's camera-frame x/y/z placed directly
into position -- test that ArduPilot's own logs show it recognizing
these as valid precision-landing input before trusting this in flight.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32, Float64, String
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import State as MavrosState, LandingTarget

STATE_INACTIVE = 'INACTIVE'
STATE_ACTIVE = 'ACTIVE'

TRIGGER_TASK_TYPE = 'RETURN'
TRIGGER_MODE = 'LAND'


class PrecisionLandingNode(Node):
    def __init__(self):
        super().__init__('precision_landing_node')

        self.declare_parameter('drone_id', 'drone1')
        self.declare_parameter('servo_down_rad', 1.5708)
        self.declare_parameter('servo_front_rad', 0.0)
        self.declare_parameter('tag_lost_grace_period_sec', 1.0)

        gp = lambda name: self.get_parameter(name).value
        self.drone_id = gp('drone_id')
        self.servo_down_rad = gp('servo_down_rad')
        self.servo_front_rad = gp('servo_front_rad')
        self.tag_lost_grace_period_sec = gp('tag_lost_grace_period_sec')

        self.state = STATE_INACTIVE
        self._current_task_type = ''
        self._mavros_mode = ''
        self._tag_visible = False
        self._last_tag_seen_time = None
        self._tag_pose = None
        self._lidar_altitude = None  # cross-check/logging only -- see docstring

        qos = QoSProfile(depth=10,
                          reliability=QoSReliabilityPolicy.BEST_EFFORT,
                          history=QoSHistoryPolicy.KEEP_LAST)

        self.create_subscription(String, 'drone_controller/current_task_type',
                                  self._task_type_cb, 10)
        self.create_subscription(MavrosState, 'state', self._mavros_state_cb, qos)
        self.create_subscription(Bool, 'drone_controller/apriltag/detected',
                                  self._detected_cb, 10)
        self.create_subscription(PoseStamped, 'drone_controller/apriltag/pose',
                                  self._pose_cb, 10)
        self.create_subscription(LaserScan, 'obstacle_lidar', self._lidar_cb, qos)

        self.detector_enable_pub = self.create_publisher(Bool, 'apriltag/enable', 10)
        self.servo_cmd_pub = self.create_publisher(Float64, 'lidar_servo_cmd', 10)
        self.landing_target_pub = self.create_publisher(
            LandingTarget, 'landing_target/raw', 10)

        self.create_timer(0.1, self._control_loop)  # 10 Hz, lightweight

        self.get_logger().info(
            f'[{self.drone_id}] precision_landing_node ready '
            f'(triggers on task_type={TRIGGER_TASK_TYPE} + mode={TRIGGER_MODE})')

    # ------------------------------------------------------------------
    def _task_type_cb(self, msg: String):
        self._current_task_type = msg.data

    def _mavros_state_cb(self, msg: MavrosState):
        self._mavros_mode = msg.mode

    def _detected_cb(self, msg: Bool):
        self._tag_visible = msg.data
        if msg.data:
            self._last_tag_seen_time = self.get_clock().now()

    def _pose_cb(self, msg: PoseStamped):
        self._tag_pose = msg.pose

    def _lidar_cb(self, msg: LaserScan):
        if not msg.ranges:
            return
        r = msg.ranges[0]
        if r == r and r not in (float('inf'), float('-inf')):  # finite check
            self._lidar_altitude = r

    # ------------------------------------------------------------------
    def _should_be_active(self):
        return (self._current_task_type == TRIGGER_TASK_TYPE and
                self._mavros_mode == TRIGGER_MODE)

    def _set_active(self, active):
        if active == (self.state == STATE_ACTIVE):
            return
        self.state = STATE_ACTIVE if active else STATE_INACTIVE
        self.get_logger().info(f'[{self.drone_id}] precision landing -> {self.state}')

        enable_msg = Bool()
        enable_msg.data = active
        self.detector_enable_pub.publish(enable_msg)

        servo_msg = Float64()
        servo_msg.data = self.servo_down_rad if active else self.servo_front_rad
        self.servo_cmd_pub.publish(servo_msg)

    def _control_loop(self):
        want_active = self._should_be_active()
        if want_active != (self.state == STATE_ACTIVE):
            self._set_active(want_active)

        if self.state != STATE_ACTIVE:
            return

        if self._last_tag_seen_time is not None:
            elapsed = (self.get_clock().now() - self._last_tag_seen_time).nanoseconds / 1e9
            fresh = elapsed <= self.tag_lost_grace_period_sec
        else:
            fresh = False

        if not fresh or self._tag_pose is None:
            return  # nothing new to relay this cycle; ArduPilot holds its last known target

        self._publish_landing_target()

    def _publish_landing_target(self):
        msg = LandingTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{self.drone_id}/camera_optical_frame'

        msg.target_num = 0
        msg.frame = LandingTarget.BODY_NED
        msg.pose.position.x = self._tag_pose.position.x
        msg.pose.position.y = self._tag_pose.position.y
        msg.pose.position.z = self._tag_pose.position.z
        msg.size[0] = 0.0
        msg.size[1] = 0.0
        msg.distance = self._tag_pose.position.z
        msg.type = LandingTarget.VISION_FIDUCIAL

        self.landing_target_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PrecisionLandingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
