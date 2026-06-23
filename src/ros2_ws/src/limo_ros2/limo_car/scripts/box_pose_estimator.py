#!/usr/bin/python3
"""
box_pose_estimator — turn the blue box into a 3D pose for MoveIt (Stage 2).

Reuses the proven HSV+depth detection from box_follower, but instead of driving
the base it DEPROJECTS the box's pixel + depth into a 3D point using the camera
intrinsics, transforms it into the arm's planning frame (base_link) via TF2, and
publishes it as a geometry_msgs/PoseStamped on /box_pose.

The orchestrator (pick_orchestrator.py) subscribes to /box_pose and plans a
top-down grasp to it. Keeping perception this simple (no PCL / RANSAC / MTC) is a
deliberate choice — see the project notes on the other thesis's over-engineering.

Pipeline:
  /rgb/image_raw (bgr) ── HSV mask ── closest blue contour centroid (u, v)
  /depth_camera/depth/image_raw (32FC1) ── depth Z at (u, v)
  /depth_camera/depth/camera_info ── fx, fy, cx, cy (intrinsics)
        X = (u-cx)·Z/fx,  Y = (v-cy)·Z/fy,  Z = Z      (optical frame, REP 103)
  TF2: camera optical frame → base_link
        publish /box_pose (PoseStamped, orientation = identity; the grasp
        orientation is chosen by the orchestrator, not here)
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import Marker

import tf2_ros
from tf2_geometry_msgs import do_transform_point

from cv_bridge import CvBridge
import cv2
import numpy as np


# HSV range for the vivid blue Gazebo box (same as box_follower / limo_pick_place)
BLUE_HSV_LO = np.array([110, 150,  50])
BLUE_HSV_HI = np.array([130, 255, 255])

MIN_CONTOUR_AREA = 30          # px², ignore noise
TARGET_FRAME     = 'base_link'  # MoveIt arm-planning frame


class BoxPoseEstimator(Node):

    def __init__(self):
        super().__init__('box_pose_estimator')

        self.bridge = CvBridge()
        self._depth = None          # latest depth image (np.float32)
        self._fx = self._fy = None  # intrinsics
        self._cx = self._cy = None
        self._depth_frame = None    # optical frame id from the depth image header

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.create_subscription(CameraInfo, '/depth_camera/depth/camera_info',
                                 self._caminfo_cb, 10)
        self.create_subscription(Image, '/depth_camera/depth/image_raw',
                                 self._depth_cb, 10)
        self.create_subscription(Image, '/rgb/image_raw',
                                 self._rgb_cb, 10)

        # Publishers
        self.pose_pub   = self.create_publisher(PoseStamped, '/box_pose', 10)
        self.marker_pub = self.create_publisher(Marker, '/box_pose_marker', 10)

        # Latch the last good detection in the global 'map' frame and republish it at 20 Hz,
        # transformed dynamically to base_link so the coordinate remains accurate as the robot moves.
        self._last_pose_map = None
        self.create_timer(0.05, self._publish_latched)

        self.get_logger().info('box_pose_estimator started — waiting for camera…')

    # ── Sensor callbacks ──────────────────────────────────────────────────────

    def _caminfo_cb(self, msg: CameraInfo):
        # K = [fx 0 cx; 0 fy cy; 0 0 1]
        self._fx, self._fy = msg.k[0], msg.k[4]
        self._cx, self._cy = msg.k[2], msg.k[5]

    def _depth_cb(self, msg: Image):
        try:
            self._depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')
            self._depth_frame = msg.header.frame_id
        except Exception as e:
            self.get_logger().warn(f'depth convert failed: {e}', throttle_duration_sec=5.0)

    def _rgb_cb(self, msg: Image):
        if self._depth is None or self._fx is None or self._depth_frame is None:
            self.get_logger().info('Waiting for depth + camera_info…',
                                   throttle_duration_sec=3.0)
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'rgb convert failed: {e}', throttle_duration_sec=5.0)
            return

        h, w = bgr.shape[:2]
        depth = self._depth
        # Guard against rgb/depth resolution mismatch.
        if depth.shape[0] != h or depth.shape[1] != w:
            self.get_logger().warn(
                f'rgb {w}x{h} vs depth {depth.shape[1]}x{depth.shape[0]} size mismatch',
                throttle_duration_sec=5.0)
            return

        uv = self._detect_box_pixel(bgr, depth, h, w)
        if uv is None:
            self.get_logger().info('No blue box visible.', throttle_duration_sec=3.0)
            return
        u, v, z = uv

        # Deproject pixel + depth → 3D point in the camera optical frame (REP 103).
        x = (u - self._cx) * z / self._fx
        y = (v - self._cy) * z / self._fy
        pt = PointStamped()
        pt.header.frame_id = self._depth_frame
        pt.header.stamp = rclpy.time.Time().to_msg()  # latest available transform
        pt.point.x, pt.point.y, pt.point.z = float(x), float(y), float(z)

        # Transform into the global static 'map' frame.
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', self._depth_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF {self._depth_frame}→map unavailable: {e}',
                                   throttle_duration_sec=3.0)
            return

        pt_map = do_transform_point(pt, tf)
        self._last_pose_map = pt_map   # latch stationary world point

        # Transform to base_link immediately to publish and log
        try:
            tf_base = self.tf_buffer.lookup_transform(
                TARGET_FRAME, 'map',
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
            pt_base = do_transform_point(pt_map, tf_base)

            pose = PoseStamped()
            pose.header.frame_id = TARGET_FRAME
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position = pt_base.point
            pose.pose.orientation.w = 1.0   # location only; grasp orientation set by orchestrator
            self.pose_pub.publish(pose)
            self._publish_marker(pose)

            self.get_logger().info(
                f'box @ base_link: x={pt_base.point.x:.3f} y={pt_base.point.y:.3f} '
                f'z={pt_base.point.z:.3f} (cam Z={z:.3f})', throttle_duration_sec=1.0)
        except Exception as e:
            self.get_logger().warn(f'Failed immediate base_link transform: {e}', throttle_duration_sec=3.0)

    # ── Detection helper ──────────────────────────────────────────────────────

    def _detect_box_pixel(self, bgr, depth, h, w):
        """Return (u, v, median_depth) of the closest blue blob, or None."""
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, BLUE_HSV_LO, BLUE_HSV_HI)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None  # (z, u, v)
        for c in contours:
            if cv2.contourArea(c) < MIN_CONTOUR_AREA:
                continue
            m = cv2.moments(c)
            if m['m00'] == 0:
                continue
            u = int(m['m10'] / m['m00'])
            v = int(m['m01'] / m['m00'])
            depths = []
            for dy in range(-2, 3):
                for dx in range(-2, 3):
                    ny, nx = v + dy, u + dx
                    if 0 <= ny < h and 0 <= nx < w:
                        d = depth[ny, nx]
                        if np.isfinite(d) and d > 0.05:
                            depths.append(d)
            if depths:
                z = float(np.median(depths))
                if best is None or z < best[0]:
                    best = (z, u, v)
        if best is None:
            return None
        return best[1], best[2], best[0]

    def _publish_latched(self):
        """Republish the last known box pose at 20 Hz, dynamically transformed from
        map to base_link so the coordinate remains accurate as the robot moves."""
        if self._last_pose_map is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                TARGET_FRAME, 'map',
                rclpy.time.Time(), timeout=Duration(seconds=0.05))
            pt_base = do_transform_point(self._last_pose_map, tf)

            pose = PoseStamped()
            pose.header.frame_id = TARGET_FRAME
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position = pt_base.point
            pose.pose.orientation.w = 1.0

            self.pose_pub.publish(pose)
            self._publish_marker(pose)
        except Exception as e:
            self.get_logger().debug(f'TF map→{TARGET_FRAME} transform failed: {e}')

    def _publish_marker(self, pose: PoseStamped):
        m = Marker()
        m.header = pose.header
        m.ns = 'box_pose'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose = pose.pose
        m.scale.x = m.scale.y = m.scale.z = 0.05
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.6, 1.0, 0.9
        self.marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = BoxPoseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
