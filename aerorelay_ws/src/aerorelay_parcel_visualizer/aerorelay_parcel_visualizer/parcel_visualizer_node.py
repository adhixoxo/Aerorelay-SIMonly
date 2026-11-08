#!/usr/bin/env python3
"""
parcel_visualizer_node.py

Moves the 'parcel_box' Gazebo model to visually track the parcel's real
state: sits at the pickup marker, follows drone1 while PICKED_UP, sits at
the hub during TRANSFER_PACKAGE, follows drone2 while ON_DRONE_2, then
sits at the delivery marker once DELIVERED.

Uses `gz service` via subprocess to call Gazebo's set_pose service --
this is the most portable way to move a model from outside the simulation
without writing a native Gazebo plugin. Verify the service name with
`gz service -l | grep set_pose` if this doesn't move anything.
"""

import subprocess
import threading

import rclpy
from rclpy.node import Node

from aerorelay_msgs.msg import ParcelState, DroneState

WORLD_NAME = 'iris_runway'
MODEL_NAME = 'parcel_box'
SET_POSE_SERVICE = f'/world/{WORLD_NAME}/set_pose'


class ParcelVisualizer(Node):
    def __init__(self):
        super().__init__('parcel_visualizer_node')

        self.declare_parameter('drone1_id', 'drone1')
        self.declare_parameter('drone2_id', 'drone2')
        self.drone1_id = self.get_parameter('drone1_id').get_parameter_value().string_value
        self.drone2_id = self.get_parameter('drone2_id').get_parameter_value().string_value

        self._lock = threading.Lock()
        self.parcel_state = ParcelState.STATE_WAITING
        self.pickup = (5.0, 0.0, 0.2)
        self.delivery = (-5.0, 5.0, 0.2)
        self.drone1_pos = (0.0, 0.0, 0.2)
        self.drone2_pos = (5.0, 0.0, 0.2)
        self.hub_pos = (0.0, 0.0, 0.2)
        self._last_moved_pos = None

        self.create_subscription(ParcelState, '/hub/parcel_state', self._parcel_cb, 10)
        self.create_subscription(
            DroneState, f'/{self.drone1_id}/drone_controller/state', self._drone1_cb, 10)
        self.create_subscription(
            DroneState, f'/{self.drone2_id}/drone_controller/state', self._drone2_cb, 10)

        self.create_timer(1.0, self._tick)
        self.get_logger().info('parcel_visualizer ready')

    def _parcel_cb(self, msg: ParcelState):
        with self._lock:
            self.parcel_state = msg.state
            new_pickup = None
            new_delivery = None
            if msg.pickup_location.x != 0.0 or msg.pickup_location.y != 0.0:
                new_pickup = (msg.pickup_location.x, msg.pickup_location.y, 0.2)
                self.pickup = new_pickup
            if msg.delivery_location.x != 0.0 or msg.delivery_location.y != 0.0:
                new_delivery = (msg.delivery_location.x, msg.delivery_location.y, 0.2)
                self.delivery = new_delivery
        if new_pickup is not None:
            self._set_model_pose(new_pickup, model_name='pickup_marker')
        if new_delivery is not None:
            self._set_model_pose(new_delivery, model_name='delivery_marker')

    def _drone1_cb(self, msg: DroneState):
        with self._lock:
            self.drone1_pos = (msg.position.x, msg.position.y, max(0.2, msg.position.z))

    def _drone2_cb(self, msg: DroneState):
        with self._lock:
            self.drone2_pos = (msg.position.x, msg.position.y, max(0.2, msg.position.z))

    def _tick(self):
        with self._lock:
            state = self.parcel_state
            pos = self.pickup  # default

            if state in (ParcelState.STATE_WAITING, ParcelState.STATE_ASSIGNED):
                pos = self.pickup
            elif state == ParcelState.STATE_PICKED_UP:
                pos = self.drone1_pos
            elif state == ParcelState.STATE_AT_HUB:
                pos = self.hub_pos
            elif state == ParcelState.STATE_ON_DRONE_2:
                pos = self.drone2_pos
            elif state in (ParcelState.STATE_DELIVERED, ParcelState.STATE_COMPLETED):
                pos = self.delivery

        # Only issue a gz service call when the target position actually
        # changed -- calling this on every 0.3s tick regardless (300+ times
        # per mission) was spawning that many subprocesses against an
        # already CPU-saturated Gazebo (real_time_factor ~0.6), which is
        # what was starving the ArduPilotPlugin FDM socket mid-mission.
        if pos != self._last_moved_pos:
            threading.Thread(
                target=self._set_model_pose, args=(pos,), daemon=True
            ).start()
            self._last_moved_pos = pos

    def _set_model_pose(self, pos, model_name=None):
        name = model_name or MODEL_NAME
        x, y, z = pos
        req = f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}'
        try:
            subprocess.run(
                ['gz', 'service', '-s', SET_POSE_SERVICE,
                 '--reqtype', 'gz.msgs.Pose',
                 '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '300',
                 '--req', req],
                capture_output=True, timeout=1.0,
            )
        except Exception as e:
            self.get_logger().warn(f'set_pose call failed: {e}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = ParcelVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
