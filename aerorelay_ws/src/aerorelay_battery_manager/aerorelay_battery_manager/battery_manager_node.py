#!/usr/bin/env python3
"""
battery_manager_node.py

Simulates battery drain/charge for each drone based on its reported flight
state (from drone_controller/state). Publishes a simple Float32 percentage
per drone that drone_controller subscribes to and reports back in its own
DroneState -- this node never talks to MAVROS or the hub directly.

Charging model: a drone charges only while WAITING *and* within ~1m of the
hub position (0,0) -- i.e. actually docked, not just landed somewhere on a
mission leg. This is a placeholder for real docking detection; swap in
actual dock-sensor logic here once the docking module exists.
"""

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from aerorelay_msgs.msg import DroneState

# percent-per-second drain rates by flight state
DRAIN_RATES = {
    DroneState.STATE_TAKEOFF: 0.20,
    DroneState.STATE_NAVIGATE: 0.10,
    DroneState.STATE_LAND: 0.12,
    DroneState.STATE_READY: 0.02,
    DroneState.STATE_IDLE: 0.005,
}
STANDBY_DRAIN_RATE = 0.01      # WAITING, but not at hub
CHARGE_RATE = 1.0              # percent-per-second while docked at hub
HUB_DOCK_RADIUS_M = 1.0
UPDATE_HZ = 2.0


class BatteryManager(Node):
    def __init__(self):
        super().__init__('battery_manager_node')

        self.declare_parameter('drone1_id', 'drone1')
        self.declare_parameter('drone2_id', 'drone2')
        self.drone1_id = self.get_parameter('drone1_id').get_parameter_value().string_value
        self.drone2_id = self.get_parameter('drone2_id').get_parameter_value().string_value

        self._battery = {self.drone1_id: 100.0, self.drone2_id: 100.0}
        self._flight_state = {self.drone1_id: DroneState.STATE_IDLE,
                               self.drone2_id: DroneState.STATE_IDLE}
        self._position = {self.drone1_id: (0.0, 0.0), self.drone2_id: (0.0, 0.0)}
        self._last_update = time.time()

        self.create_subscription(
            DroneState, f'/{self.drone1_id}/drone_controller/state',
            lambda msg: self._state_cb(self.drone1_id, msg), 10)
        self.create_subscription(
            DroneState, f'/{self.drone2_id}/drone_controller/state',
            lambda msg: self._state_cb(self.drone2_id, msg), 10)

        self.publishers_ = {
            self.drone1_id: self.create_publisher(Float32, f'/{self.drone1_id}/battery/percent', 10),
            self.drone2_id: self.create_publisher(Float32, f'/{self.drone2_id}/battery/percent', 10),
        }

        self.create_timer(1.0 / UPDATE_HZ, self._tick)

        self.get_logger().info(
            f'battery_manager ready for {self.drone1_id}, {self.drone2_id}')

    def _state_cb(self, drone_id, msg: DroneState):
        self._flight_state[drone_id] = msg.state
        self._position[drone_id] = (msg.position.x, msg.position.y)

    def _tick(self):
        now = time.time()
        dt = now - self._last_update
        self._last_update = now

        for drone_id in (self.drone1_id, self.drone2_id):
            state = self._flight_state[drone_id]
            x, y = self._position[drone_id]
            at_hub = math.hypot(x, y) <= HUB_DOCK_RADIUS_M

            if state == DroneState.STATE_WAITING and at_hub:
                delta = CHARGE_RATE * dt
            elif state == DroneState.STATE_WAITING:
                delta = -STANDBY_DRAIN_RATE * dt
            else:
                delta = -DRAIN_RATES.get(state, STANDBY_DRAIN_RATE) * dt

            self._battery[drone_id] = max(0.0, min(100.0, self._battery[drone_id] + delta))

            msg = Float32()
            msg.data = self._battery[drone_id]
            self.publishers_[drone_id].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
