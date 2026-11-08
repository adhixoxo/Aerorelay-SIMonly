#!/usr/bin/env python3
"""
docking_manager_node.py

Assigns each drone a fixed dock position at the hub and publishes it
periodically. mission_manager subscribes to this instead of using a
hardcoded shared HUB_POSITION -- fixes the "both drones return to the
same point and collide" bug. battery_manager also subscribes, using
dock_radius to decide when a drone is close enough to charge.

Dock positions are launch parameters, not hardcoded, so they can be
changed to match a different world file without touching this code --
this is also the natural extension point for multiple hubs later.
"""

import rclpy
from rclpy.node import Node

from aerorelay_msgs.msg import DockPosition

PUBLISH_HZ = 1.0
DEFAULT_DOCK_RADIUS_M = 1.0


class DockingManager(Node):
    def __init__(self):
        super().__init__('docking_manager_node')

        self.declare_parameter('drone1_id', 'drone1')
        self.declare_parameter('drone2_id', 'drone2')
        self.declare_parameter('drone1_dock_x', 0.0)
        self.declare_parameter('drone1_dock_y', 0.0)
        self.declare_parameter('drone1_dock_z', 0.0)
        self.declare_parameter('drone2_dock_x', 5.0)
        self.declare_parameter('drone2_dock_y', 0.0)
        self.declare_parameter('drone2_dock_z', 0.0)
        self.declare_parameter('dock_radius', DEFAULT_DOCK_RADIUS_M)

        gp = lambda name: self.get_parameter(name).get_parameter_value()

        self.docks = {
            gp('drone1_id').string_value: (
                gp('drone1_dock_x').double_value,
                gp('drone1_dock_y').double_value,
                gp('drone1_dock_z').double_value,
            ),
            gp('drone2_id').string_value: (
                gp('drone2_dock_x').double_value,
                gp('drone2_dock_y').double_value,
                gp('drone2_dock_z').double_value,
            ),
        }
        self.dock_radius = gp('dock_radius').double_value

        self.publisher = self.create_publisher(DockPosition, '/hub/dock_positions', 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._publish_docks)

        for drone_id, (x, y, z) in self.docks.items():
            self.get_logger().info(f'{drone_id} dock assigned at ({x}, {y}, {z})')

    def _publish_docks(self):
        for drone_id, (x, y, z) in self.docks.items():
            msg = DockPosition()
            msg.drone_id = drone_id
            msg.x = x
            msg.y = y
            msg.z = z
            msg.dock_radius = self.dock_radius
            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DockingManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
