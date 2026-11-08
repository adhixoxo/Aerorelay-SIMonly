#!/usr/bin/env python3
"""
Autonomous square-flight mission for ArduCopter SITL via MAVROS.
Requires mavros running and connected (check `ros2 topic echo /mavros/state`).
"""
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
import time


class SquareMission(Node):
    def __init__(self):
        super().__init__('square_mission')

        self.state = State()
        self.create_subscription(State, '/mavros/state', self._state_cb, 10)

        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')

        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)

        for client, name in [(self.arm_client, 'arming'),
                              (self.mode_client, 'set_mode'),
                              (self.takeoff_client, 'takeoff'),
                              (self.land_client, 'land')]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {name} service...')

    def _state_cb(self, msg):
        self.state = msg

    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def arm(self, value=True):
        req = CommandBool.Request()
        req.value = value
        future = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def takeoff(self, altitude):
        req = CommandTOL.Request()
        req.altitude = altitude
        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def land(self):
        req = CommandTOL.Request()
        future = self.land_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_setpoint(self, x, y, z, duration_sec=5.0, rate_hz=10):
        """Publish a position setpoint repeatedly for GUIDED mode to hold onto."""
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0

        cycles = int(duration_sec * rate_hz)
        for _ in range(cycles):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.setpoint_pub.publish(msg)
            time.sleep(1.0 / rate_hz)

    def run_mission(self):
        self.get_logger().info('Setting mode to GUIDED...')
        self.set_mode('GUIDED')
        time.sleep(2)

        self.get_logger().info('Arming...')
        self.arm(True)
        time.sleep(2)

        self.get_logger().info('Taking off to 5m...')
        self.takeoff(5.0)
        time.sleep(8)  # allow climb to altitude

        altitude = 5.0
        side = 5.0  # meters per side of the square

        square_corners = [
            (side, 0.0, altitude),
            (side, side, altitude),
            (0.0, side, altitude),
            (0.0, 0.0, altitude),
        ]

        for i, (x, y, z) in enumerate(square_corners):
            self.get_logger().info(f'Flying to corner {i+1}: ({x}, {y}, {z})')
            self.send_setpoint(x, y, z, duration_sec=6.0)

        self.get_logger().info('Landing...')
        self.land()


def main(args=None):
    rclpy.init(args=args)
    node = SquareMission()
    try:
        node.run_mission()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
