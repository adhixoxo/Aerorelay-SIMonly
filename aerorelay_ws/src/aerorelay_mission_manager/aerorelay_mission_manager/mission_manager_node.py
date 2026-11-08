#!/usr/bin/env python3
"""
mission_manager_node.py

The AeroRelay Hub's central controller. Runs as a standalone node (not
namespaced under a drone) and orchestrates the full relay scenario:

  MissionRequest received
    -> assign drone1 to fly to pickup_location
    -> wait for drone1 to land (WAITING)
    -> command drone1 to return to hub
    -> wait for drone1 to land at hub (WAITING)
    -> transfer parcel to drone2 (software flag flip)
    -> assign drone2 to fly to delivery_location
    -> wait for drone2 to land (WAITING)
    -> command drone2 to return to hub
    -> wait for drone2 to land at hub (WAITING)
    -> mission complete

This node never touches MAVROS or Gazebo directly -- it only calls each
drone_controller's AssignMission service and watches its DroneState. That
boundary is what lets you add more drones or swap in real hardware later
without changing this file.
"""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Point

from aerorelay_msgs.msg import DroneState, ParcelState, MissionRequest, HubStatus, DockPosition, DockPosition
from aerorelay_msgs.srv import AssignMission


DRONE_STATE_POLL_INTERVAL_SEC = 0.5
DRONE_STATE_TIMEOUT_SEC = 120.0

# Fallback used only if docking_manager hasn't published yet for a drone
DEFAULT_HUB_POSITION = Point(x=0.0, y=0.0, z=0.0)


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager_node')

        self.declare_parameter('drone1_id', 'drone1')
        self.declare_parameter('drone2_id', 'drone2')
        self.drone1_id = self.get_parameter('drone1_id').get_parameter_value().string_value
        self.drone2_id = self.get_parameter('drone2_id').get_parameter_value().string_value

        self._lock = threading.Lock()
        self._busy = False
        self.hub_state = HubStatus.STATE_IDLE
        self.active_mission_id = ''
        self.parcel_state = ParcelState.STATE_WAITING
        self._active_pickup = Point()
        self._active_delivery = Point()

        self._drone_states = {}
        self._dock_positions = {}

        self.create_subscription(
            DockPosition, '/hub/dock_positions', self._dock_position_cb, 10)
        self._dock_positions = {}

        self.create_subscription(
            DockPosition, '/hub/dock_positions', self._dock_position_cb, 10)

        cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            DroneState, f'/{self.drone1_id}/drone_controller/state',
            lambda msg: self._drone_state_cb(self.drone1_id, msg), 10)
        self.create_subscription(
            DroneState, f'/{self.drone2_id}/drone_controller/state',
            lambda msg: self._drone_state_cb(self.drone2_id, msg), 10)

        self.create_subscription(
            MissionRequest, 'mission_requests', self._mission_request_cb, 10)

        self.assign_clients = {
            self.drone1_id: self.create_client(
                AssignMission, f'/{self.drone1_id}/drone_controller/assign_mission',
                callback_group=cb_group),
            self.drone2_id: self.create_client(
                AssignMission, f'/{self.drone2_id}/drone_controller/assign_mission',
                callback_group=cb_group),
        }

        self.hub_status_pub = self.create_publisher(HubStatus, 'hub/status', 10)
        self.parcel_state_pub = self.create_publisher(ParcelState, 'hub/parcel_state', 10)

        self.create_timer(0.5, self._publish_status)

        self.get_logger().info(
            f'mission_manager ready. drone1={self.drone1_id}, drone2={self.drone2_id}')

    def _drone_state_cb(self, drone_id, msg: DroneState):
        self._drone_states[drone_id] = msg

    def _dock_position_cb(self, msg: DockPosition):
        self._dock_positions[msg.drone_id] = Point(x=msg.x, y=msg.y, z=msg.z)

    def _dock_position_for(self, drone_id):
        return self._dock_positions.get(drone_id, DEFAULT_HUB_POSITION)

    def _dock_position_cb(self, msg: DockPosition):
        self._dock_positions[msg.drone_id] = Point(x=msg.x, y=msg.y, z=msg.z)

    def _dock_position_for(self, drone_id):
        return self._dock_positions.get(drone_id, DEFAULT_HUB_POSITION)

    def _set_hub_state(self, state):
        with self._lock:
            self.hub_state = state
        self.get_logger().info(f'hub_state -> {state}')

    def _set_parcel_state(self, state):
        with self._lock:
            self.parcel_state = state
        self.get_logger().info(f'parcel_state -> {state}')

    def _publish_status(self):
        msg = HubStatus()
        with self._lock:
            msg.state = self.hub_state
            msg.active_mission_id = self.active_mission_id
        msg.drone1_id = self.drone1_id
        msg.drone2_id = self.drone2_id
        msg.stamp = self.get_clock().now().to_msg()
        self.hub_status_pub.publish(msg)

        parcel_msg = ParcelState()
        parcel_msg.parcel_id = self.active_mission_id or ''
        with self._lock:
            parcel_msg.state = self.parcel_state
        parcel_msg.mission_id = self.active_mission_id
        parcel_msg.pickup_location = self._active_pickup
        parcel_msg.delivery_location = self._active_delivery
        parcel_msg.stamp = self.get_clock().now().to_msg()
        self.parcel_state_pub.publish(parcel_msg)

    def _mission_request_cb(self, request: MissionRequest):
        with self._lock:
            if self._busy:
                self.get_logger().warn(
                    f'Ignoring mission {request.mission_id}: hub is busy '
                    f'with {self.active_mission_id}')
                return
            self._busy = True
            self.active_mission_id = request.mission_id

        thread = threading.Thread(
            target=self._run_mission,
            args=(request.mission_id, request.pickup_location, request.delivery_location),
            daemon=True,
        )
        thread.start()

    def _run_mission(self, mission_id, pickup_location, delivery_location):
        self._active_pickup = pickup_location
        self._active_delivery = delivery_location
        self.get_logger().info(f'Mission {mission_id} received.')
        self._set_hub_state(HubStatus.STATE_MISSION_RECEIVED)
        self._set_parcel_state(ParcelState.STATE_ASSIGNED)

        self._set_hub_state(HubStatus.STATE_ASSIGN_DRONE)
        if not self._assign(self.drone1_id, mission_id, pickup_location, 'PICKUP'):
            self._abort_mission('drone1 rejected pickup assignment')
            return

        self._set_hub_state(HubStatus.STATE_WAIT_FOR_RETURN)
        if not self._wait_for_drone_state(self.drone1_id, DroneState.STATE_WAITING):
            self._abort_mission('drone1 did not reach pickup location in time')
            return
        self._set_parcel_state(ParcelState.STATE_PICKED_UP)

        if not self._assign(self.drone1_id, mission_id, self._dock_position_for(self.drone1_id), 'RETURN'):
            self._abort_mission('drone1 rejected return-to-hub assignment')
            return
        if not self._wait_for_drone_state(self.drone1_id, DroneState.STATE_WAITING):
            self._abort_mission('drone1 did not return to hub in time')
            return

        self._set_hub_state(HubStatus.STATE_TRANSFER_PACKAGE)
        self._set_parcel_state(ParcelState.STATE_AT_HUB)
        time.sleep(1.0)
        self._set_parcel_state(ParcelState.STATE_ON_DRONE_2)

        self._set_hub_state(HubStatus.STATE_ASSIGN_SECOND_DRONE)
        if not self._assign(self.drone2_id, mission_id, delivery_location, 'DELIVER'):
            self._abort_mission('drone2 rejected delivery assignment')
            return

        self._set_hub_state(HubStatus.STATE_WAIT_FOR_DELIVERY)
        if not self._wait_for_drone_state(self.drone2_id, DroneState.STATE_WAITING):
            self._abort_mission('drone2 did not reach delivery location in time')
            return
        self._set_parcel_state(ParcelState.STATE_DELIVERED)

        if not self._assign(self.drone2_id, mission_id, self._dock_position_for(self.drone2_id), 'RETURN'):
            self._abort_mission('drone2 rejected return-to-hub assignment')
            return
        if not self._wait_for_drone_state(self.drone2_id, DroneState.STATE_WAITING):
            self._abort_mission('drone2 did not return to hub in time')
            return

        self._set_parcel_state(ParcelState.STATE_COMPLETED)
        self._set_hub_state(HubStatus.STATE_MISSION_COMPLETE)
        self.get_logger().info(f'Mission {mission_id} complete.')

        time.sleep(1.0)
        self._set_hub_state(HubStatus.STATE_IDLE)
        with self._lock:
            self._busy = False
            self.active_mission_id = ''

    def _abort_mission(self, reason):
        self.get_logger().error(f'Mission aborted: {reason}')
        self._set_hub_state(HubStatus.STATE_IDLE)
        with self._lock:
            self._busy = False
            self.active_mission_id = ''

    def _assign(self, drone_id, mission_id, target_location, task_type):
        client = self.assign_clients[drone_id]
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'assign_mission service unavailable for {drone_id}')
            return False

        req = AssignMission.Request()
        req.mission_id = mission_id
        req.parcel_id = mission_id
        req.target_location = target_location
        req.task_type = task_type

        future = client.call_async(req)
        start = time.time()
        while not future.done():
            if time.time() - start > 10.0:
                self.get_logger().error(f'assign_mission call to {drone_id} timed out')
                return False
            time.sleep(0.05)

        result = future.result()
        if result is None or not result.accepted:
            reason = result.reason if result else 'no response'
            self.get_logger().error(f'{drone_id} rejected mission: {reason}')
            return False
        return True

    def _wait_for_drone_state(self, drone_id, target_state):
        # Record the moment we start waiting. A stale cached DroneState left
        # over from the *previous* leg can already equal target_state (e.g.
        # both legs end in WAITING) -- if we only checked the value, we'd
        # return True instantly without the drone ever having flown this
        # leg. Requiring the message's own stamp to be newer than
        # wait_start_ns closes that race.
        wait_start_ns = self.get_clock().now().nanoseconds
        start = time.time()
        while time.time() - start < DRONE_STATE_TIMEOUT_SEC:
            state_msg = self._drone_states.get(drone_id)
            if state_msg is not None and state_msg.state == target_state:
                msg_ns = state_msg.stamp.sec * 1_000_000_000 + state_msg.stamp.nanosec
                if msg_ns > wait_start_ns:
                    return True
            time.sleep(DRONE_STATE_POLL_INTERVAL_SEC)
        return False


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
