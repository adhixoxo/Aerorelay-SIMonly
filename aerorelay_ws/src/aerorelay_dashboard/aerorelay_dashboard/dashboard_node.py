#!/usr/bin/env python3
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from aerorelay_msgs.msg import DroneState, HubStatus, ParcelState


class SharedFleetState:
    def __init__(self):
        self._lock = threading.Lock()
        self.hub_state = HubStatus.STATE_IDLE
        self.parcel_state = ParcelState.STATE_WAITING
        self.drones = {}

    def set_hub_state(self, state):
        with self._lock:
            self.hub_state = state

    def set_parcel_state(self, state):
        with self._lock:
            self.parcel_state = state

    def set_drone_state(self, drone_id, msg: DroneState):
        with self._lock:
            self.drones[drone_id] = {
                'drone_id': drone_id,
                'state': msg.state,
                'current_mission_id': msg.current_mission_id,
                'battery_percent': msg.battery_percent,
                'position': {'x': msg.position.x, 'y': msg.position.y, 'z': msg.position.z},
                'healthy': msg.healthy,
                'health_message': msg.health_message,
            }

    def snapshot(self):
        with self._lock:
            return {'hub_state': self.hub_state, 'parcel_state': self.parcel_state,
                     'drones': dict(self.drones)}


def make_handler(shared_state, web_dir):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def do_GET(self):
            if self.path == '/api/state':
                body = json.dumps(shared_state.snapshot()).encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path in ('/', '/index.html'):
                body = (web_dir / 'index.html').read_bytes()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            self.send_response(404)
            self.end_headers()
    return Handler


class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')
        self.declare_parameter('drone1_id', 'drone1')
        self.declare_parameter('drone2_id', 'drone2')
        self.declare_parameter('port', 8080)

        self.drone1_id = self.get_parameter('drone1_id').get_parameter_value().string_value
        self.drone2_id = self.get_parameter('drone2_id').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value

        self.shared_state = SharedFleetState()

        self.create_subscription(HubStatus, '/hub/status',
            lambda msg: self.shared_state.set_hub_state(msg.state), 10)
        self.create_subscription(ParcelState, '/hub/parcel_state',
            lambda msg: self.shared_state.set_parcel_state(msg.state), 10)
        self.create_subscription(DroneState, f'/{self.drone1_id}/drone_controller/state',
            lambda msg: self.shared_state.set_drone_state(self.drone1_id, msg), 10)
        self.create_subscription(DroneState, f'/{self.drone2_id}/drone_controller/state',
            lambda msg: self.shared_state.set_drone_state(self.drone2_id, msg), 10)

        web_dir = Path(get_package_share_directory('aerorelay_dashboard')) / 'web'
        handler = make_handler(self.shared_state, web_dir)
        self.http_server = ThreadingHTTPServer(('0.0.0.0', self.port), handler)
        threading.Thread(target=self.http_server.serve_forever, daemon=True).start()

        self.get_logger().info(
            f'Dashboard serving on http://0.0.0.0:{self.port} (tracking {self.drone1_id}, {self.drone2_id})')

    def destroy_node(self):
        self.http_server.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
