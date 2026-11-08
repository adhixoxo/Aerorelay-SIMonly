#!/usr/bin/env python3
"""
apriltag_detector_node.py

Subscribes to a drone's downward camera (image + camera_info), runs
AprilTag detection with real 6-DOF pose estimation (requires camera
intrinsics from camera_info -- pixel-only detection without real
intrinsics would give unreliable/unscaled pose), and publishes:

  drone_controller/apriltag/detected  (std_msgs/Bool)
  drone_controller/apriltag/pose      (geometry_msgs/PoseStamped, tag
                                        pose in the camera's optical frame)

This node does not make any landing decisions -- it only reports what it
sees. The precision-landing controller (next step) consumes this output
to correct descent; keeping detection and control separate means we can
test/validate the vision pipeline independently before trusting it to
fly anything.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

from pupil_apriltags import Detector

# Physical tag size in meters -- MUST match the real/simulated tag's
# actual printed/rendered size, or every pose estimate is silently
# scaled wrong. Matches the 0.4m x 0.4m ground markers in the world file.
TAG_SIZE_M = 0.4

# How long a detection is considered "fresh" before detected=False is
# republished even without a new camera frame (protects against a
# consumer trusting a stale detection if the camera feed stalls).
DETECTION_STALE_TIMEOUT_SEC = 1.0


def rotation_matrix_to_quaternion(r):
    """Convert a 3x3 rotation matrix (as returned by pupil_apriltags) to a
    quaternion (x, y, z, w), without adding a scipy dependency."""
    trace = r[0][0] + r[1][1] + r[2][2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (r[2][1] - r[1][2]) * s
        y = (r[0][2] - r[2][0]) * s
        z = (r[1][0] - r[0][1]) * s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = 2.0 * math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2])
        w = (r[2][1] - r[1][2]) / s
        x = 0.25 * s
        y = (r[0][1] + r[1][0]) / s
        z = (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = 2.0 * math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2])
        w = (r[0][2] - r[2][0]) / s
        x = (r[0][1] + r[1][0]) / s
        y = 0.25 * s
        z = (r[1][2] + r[2][1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1])
        w = (r[1][0] - r[0][1]) / s
        x = (r[0][2] + r[2][0]) / s
        y = (r[1][2] + r[2][1]) / s
        z = 0.25 * s
    return x, y, z, w


class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector_node')

        self.declare_parameter('drone_id', 'drone1')
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.detector = Detector(families='tag36h11')

        self._camera_intrinsics = None  # (fx, fy, cx, cy), set once camera_info arrives
        self._last_detection_time = None
        self._detection_enabled = False  # gated by precision_landing_node's proximity check
        self._image_wh = None

        qos = QoSProfile(depth=5,
                          reliability=QoSReliabilityPolicy.BEST_EFFORT,
                          history=QoSHistoryPolicy.KEEP_LAST)

        self.create_subscription(CameraInfo, 'camera/camera_info',
                                  self._camera_info_cb, qos)
        self.create_subscription(Image, 'camera/image_raw', self._image_cb, qos)
        self.create_subscription(Bool, 'apriltag/enable', self._enable_cb, 10)

        self.detected_pub = self.create_publisher(Bool, 'drone_controller/apriltag/detected', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'drone_controller/apriltag/pose', 10)
        # New outputs needed for large-tag/land-trigger handling:
        self.area_pub = self.create_publisher(
            Float32, 'drone_controller/apriltag/tag_area_fraction', 10)
        self.near_border_pub = self.create_publisher(
            Bool, 'drone_controller/apriltag/corners_near_border', 10)

        self.create_timer(0.2, self._check_stale)

        self.get_logger().info(f'[{self.drone_id}] apriltag_detector ready, '
                                f'waiting for camera_info before detecting')

    def _camera_info_cb(self, msg: CameraInfo):
        if self._camera_intrinsics is None:
            fx, fy = msg.k[0], msg.k[4]
            cx, cy = msg.k[2], msg.k[5]
            if fx == 0.0 or fy == 0.0:
                self.get_logger().warn(
                    f'[{self.drone_id}] camera_info received but intrinsics are zero -- '
                    f'pose estimation will be unreliable until real calibration is published')
                return
            self._camera_intrinsics = (fx, fy, cx, cy)
            self.get_logger().info(
                f'[{self.drone_id}] camera intrinsics acquired: fx={fx:.1f}, fy={fy:.1f}, '
                f'cx={cx:.1f}, cy={cy:.1f}')

    def _enable_cb(self, msg: Bool):
        self._detection_enabled = msg.data
        if not msg.data:
            self.get_logger().info(f'[{self.drone_id}] detection disabled (outside search zone)')
        else:
            self.get_logger().info(f'[{self.drone_id}] detection enabled (inside search zone)')

    def _image_cb(self, msg: Image):
        if not self._detection_enabled:
            return  # gated: skip the expensive detection call entirely outside the search zone
        if self._camera_intrinsics is None:
            return  # can't do real pose estimation without intrinsics yet
        self._image_wh = (msg.width, msg.height)

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().warn(f'[{self.drone_id}] image conversion failed: {e}',
                                    throttle_duration_sec=5.0)
            return

        detections = self.detector.detect(
            cv_image,
            estimate_tag_pose=True,
            camera_params=self._camera_intrinsics,
            tag_size=TAG_SIZE_M,
        )

        if not detections:
            return  # stale-timeout timer handles publishing detected=False

        # If multiple tags visible, use the one with the best decision margin
        # (detection confidence) rather than arbitrarily the first result.
        best = max(detections, key=lambda d: d.decision_margin)

        # --- large-tag / border handling (requirement 6/7) ---
        # pupil_apriltags returns corners as a 4x2 array in pixel coords.
        corners = best.corners
        xs = [c[0] for c in corners]
        ys = [c[1] for c in corners]
        img_w, img_h = self._image_wh
        BORDER_MARGIN_PX = 5
        near_border = (
            min(xs) <= BORDER_MARGIN_PX or max(xs) >= img_w - BORDER_MARGIN_PX or
            min(ys) <= BORDER_MARGIN_PX or max(ys) >= img_h - BORDER_MARGIN_PX
        )
        # Shoelace formula for quadrilateral area, as a fraction of image area.
        area_px = 0.5 * abs(
            xs[0]*ys[1] - xs[1]*ys[0] + xs[1]*ys[2] - xs[2]*ys[1] +
            xs[2]*ys[3] - xs[3]*ys[2] + xs[3]*ys[0] - xs[0]*ys[3]
        )
        area_fraction = area_px / float(img_w * img_h)

        area_msg = Float32()
        area_msg.data = area_fraction
        self.area_pub.publish(area_msg)

        border_msg = Bool()
        border_msg.data = bool(near_border)
        self.near_border_pub.publish(border_msg)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = f'{self.drone_id}/camera_optical_frame'

        t = best.pose_t  # 3x1 translation, meters, in camera frame
        pose_msg.pose.position.x = float(t[0][0])
        pose_msg.pose.position.y = float(t[1][0])
        pose_msg.pose.position.z = float(t[2][0])

        qx, qy, qz, qw = rotation_matrix_to_quaternion(best.pose_R)
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self.pose_pub.publish(pose_msg)

        detected_msg = Bool()
        detected_msg.data = True
        self.detected_pub.publish(detected_msg)

        self._last_detection_time = self.get_clock().now()

        self.get_logger().info(
            f'[{self.drone_id}] tag {best.tag_id} detected, margin={best.decision_margin:.1f}, '
            f'offset=({t[0][0]:.2f}, {t[1][0]:.2f}, {t[2][0]:.2f})m',
            throttle_duration_sec=1.0)

    def _check_stale(self):
        if self._last_detection_time is None:
            return
        elapsed_sec = (self.get_clock().now() - self._last_detection_time).nanoseconds / 1e9
        if elapsed_sec > DETECTION_STALE_TIMEOUT_SEC:
            msg = Bool()
            msg.data = False
            self.detected_pub.publish(msg)
            self._last_detection_time = None


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
