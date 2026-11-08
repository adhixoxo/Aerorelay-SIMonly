#!/usr/bin/env python3
"""
customer_portal_node.py

Serves a simple web form where a "customer" enters pickup/delivery
coordinates. On submit, publishes an aerorelay_msgs/MissionRequest --
this is the only interface a customer-facing app needs into the whole
AeroRelay Hub system; it never touches drones or MAVROS directly.
"""

import json
import uuid
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Point
from aerorelay_msgs.msg import MissionRequest, HubStatus

DEFAULT_ALTITUDE = 5.0


def make_handler(node, web_dir):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def do_GET(self):
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

        def do_POST(self):
            if self.path != '/api/request_delivery':
                self.send_response(404)
                self.end_headers()
                return

            length = int(self.headers.get('Content-Length', 0))
            body = json.loads(self.rfile.read(length) or b'{}')

            mission_id = str(uuid.uuid4())[:8]
            msg = MissionRequest()
            msg.mission_id = mission_id
            msg.pickup_location = Point(
                x=float(body.get('pickup_x', 0.0)),
                y=float(body.get('pickup_y', 0.0)),
                z=DEFAULT_ALTITUDE)
            msg.delivery_location = Point(
                x=float(body.get('delivery_x', 0.0)),
                y=float(body.get('delivery_y', 0.0)),
                z=DEFAULT_ALTITUDE)
            msg.requested_at = node.get_clock().now().to_msg()

            busy = node.hub_state != HubStatus.STATE_IDLE
            if busy:
                response = {'accepted': False, 'reason': f'Hub busy ({node.hub_state})'}
            else:
                node.publisher.publish(msg)
                response = {'accepted': True, 'mission_id': mission_id}

            body_bytes = json.dumps(response).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body_bytes)))
            self.end_headers()
            self.wfile.write(body_bytes)

    return Handler


class CustomerPortal(Node):
    def __init__(self):
        super().__init__('customer_portal_node')
        self.declare_parameter('port', 8090)
        self.port = self.get_parameter('port').get_parameter_value().integer_value

        self.hub_state = HubStatus.STATE_IDLE
        self.create_subscription(HubStatus, '/hub/status', self._hub_status_cb, 10)

        self.publisher = self.create_publisher(MissionRequest, '/mission_requests', 10)

        web_dir = Path(get_package_share_directory('aerorelay_customer_portal')) / 'web'
        handler = make_handler(self, web_dir)
        self.http_server = ThreadingHTTPServer(('0.0.0.0', self.port), handler)
        threading.Thread(target=self.http_server.serve_forever, daemon=True).start()

        self.get_logger().info(f'Customer portal serving on http://0.0.0.0:{self.port}')

    def _hub_status_cb(self, msg: HubStatus):
        self.hub_state = msg.state

    def destroy_node(self):
        self.http_server.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CustomerPortal()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
