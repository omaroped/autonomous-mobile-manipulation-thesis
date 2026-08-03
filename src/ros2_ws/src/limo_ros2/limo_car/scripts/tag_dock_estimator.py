#!/usr/bin/python3
"""
tag_dock_estimator.py — detect AprilTag 36h11 markers on the place table and
publish the table-centre drop point + full 6-DOF tag pose for the dock controller.

Mirrors the architecture of box_pose_estimator.py:
  - Subscribe /rgb/image_raw + /depth_camera/depth/camera_info
  - Detect the nearest visible AprilTag using cv2.aruco (no extra ROS package needed)
  - solvePnP → 6-DOF tag pose in camera optical frame
  - TF: optical frame → base_link
  - Latch last good detection in map frame, republish at 20 Hz dynamically

Published topics:
  /place_tag_pose   (geometry_msgs/PoseStamped, base_link)
      Full 6-DOF pose of the tag face. Orientation encodes the face normal direction
      — used by dock_to_tag() to compute heading error (Phase A) and range (Phase C).

  /place_drop_point (geometry_msgs/PointStamped, base_link)
      Table CENTRE in base_link: tag_pos − normal · (table_side/2) at table top z.
      Used by place_box() as the arm target (x, y). Also published as a RViz marker.

Tag layout (IDs assigned in gen_apriltag.py):
    ID 0 → North face (+Y)    ID 1 → South face (-Y)
    ID 2 → East face  (+X)    ID 3 → West face  (-X)

Parameters (live — tune with ros2 param set, no relaunch):
    tag_size   (float, default 0.08) : physical tag side length in metres
    table_side (float, default 0.10) : place table square side length in metres.
                                      MUST match place_table in final_map.world — the
                                      drop point is tag_pos - normal*(table_side/2),
                                      so a stale value aims the arm past the table.
    table_top_z (float, default 0.10): table top height above ground (world z, m)

Note on coordinate convention:
    solvePnP returns the tag pose in camera coordinates.  The +Z axis of the tag
    frame points OUT of the tag (toward the robot), so it is the face normal we need.
    After TF to base_link, the +Z column of the rotation matrix gives the outward normal.
"""

import math
import numpy as np
import cv2
import cv2.aruco as aruco

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import Marker

import tf2_ros
from tf2_geometry_msgs import do_transform_point

from cv_bridge import CvBridge

TARGET_FRAME = 'base_link'
TAG_IDS_VALID = {0, 1, 2, 3}   # IDs we placed on the four table faces


class TagDockEstimator(Node):

    def __init__(self):
        super().__init__('tag_dock_estimator')

        # ── Live params (tune with ros2 param set) ────────────────────────────
        self.declare_parameter('tag_size',    0.08)
        self.declare_parameter('table_side',  0.10)   # MUST match place_table in final_map.world
        self.declare_parameter('table_top_z', 0.10)

        # ── State ─────────────────────────────────────────────────────────────
        self._bridge        = CvBridge()
        self._fx = self._fy = self._cx = self._cy = None
        self._cam_frame     = None
        self._cam_matrix    = None
        self._dist_coeffs   = np.zeros((4, 1))  # Gazebo camera: no distortion

        # cv2 4.5.x API (older than generateImageMarker)
        self._dictionary    = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36H11)
        self._det_params    = aruco.DetectorParameters_create()

        # Latched map-frame pose (tag + drop point) — same trick as box_pose_estimator
        self._last_tag_map   = None   # PointStamped in map
        self._last_drop_map  = None   # PointStamped in map
        self._last_tag_quat  = None   # (qx,qy,qz,qw) in map frame

        # TF
        self._tf_buf = tf2_ros.Buffer()
        self._tf_lis = tf2_ros.TransformListener(self._tf_buf, self)

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(CameraInfo, '/depth_camera/depth/camera_info',
                                 self._caminfo_cb, 10)
        self.create_subscription(Image, '/rgb/image_raw', self._rgb_cb, 10)

        # ── Publishers ────────────────────────────────────────────────────────
        self._tag_pub   = self.create_publisher(PoseStamped,  '/place_tag_pose',   10)
        self._drop_pub  = self.create_publisher(PointStamped, '/place_drop_point', 10)
        self._mark_pub  = self.create_publisher(Marker,       '/place_tag_marker', 10)

        # Latch republish at 20 Hz
        self.create_timer(0.05, self._publish_latched)

        self.get_logger().info('tag_dock_estimator started — waiting for camera…')

    # ── Camera info ───────────────────────────────────────────────────────────

    def _caminfo_cb(self, msg: CameraInfo):
        self._fx, self._fy = msg.k[0], msg.k[4]
        self._cx, self._cy = msg.k[2], msg.k[5]
        self._cam_frame = msg.header.frame_id
        self._cam_matrix = np.array([[self._fx, 0, self._cx],
                                     [0, self._fy, self._cy],
                                     [0,         0,       1]], dtype=np.float64)

    # ── Main detection callback ────────────────────────────────────────────────

    def _rgb_cb(self, msg: Image):
        if self._cam_matrix is None:
            self.get_logger().info('Waiting for camera_info…', throttle_duration_sec=3.0)
            return

        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'rgb convert: {e}', throttle_duration_sec=5.0)
            return

        grey = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(grey, self._dictionary, parameters=self._det_params)

        if ids is None or len(ids) == 0:
            self.get_logger().info('No AprilTag visible.', throttle_duration_sec=3.0)
            return

        # ── Pick the tag with the largest visible area (closest / most reliable) ──
        best_idx = -1
        best_area = -1.0
        for i, tid in enumerate(ids.flatten()):
            if int(tid) not in TAG_IDS_VALID:
                continue
            c = corners[i][0]
            area = cv2.contourArea(c)
            if area > best_area:
                best_area, best_idx = area, i
        if best_idx < 0:
            return

        tag_id   = int(ids[best_idx][0])
        tag_size = float(self.get_parameter('tag_size').value)

        # ── solvePnP: tag frame with +Z pointing out of the tag face ─────────
        half = tag_size / 2.0
        obj_pts = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0],
        ], dtype=np.float64)
        img_pts = corners[best_idx][0].astype(np.float64)

        ok, rvec, tvec = cv2.solvePnP(
            obj_pts, img_pts, self._cam_matrix, self._dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return

        # Tag +Z in camera frame = outward face normal toward robot
        R_tag, _ = cv2.Rodrigues(rvec)
        normal_cam = R_tag[:, 2]           # 3rd column = tag Z axis

        # Tag origin and normal as PointStamped in camera frame
        pt_cam = PointStamped()
        pt_cam.header.frame_id = self._cam_frame
        pt_cam.header.stamp    = rclpy.time.Time().to_msg()
        pt_cam.point.x = float(tvec[0])
        pt_cam.point.y = float(tvec[1])
        pt_cam.point.z = float(tvec[2])

        # ── TF: camera → map (latch) ──────────────────────────────────────────
        try:
            tf_to_map = self._tf_buf.lookup_transform(
                'map', self._cam_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF cam→map: {e}', throttle_duration_sec=3.0)
            return

        pt_map = do_transform_point(pt_cam, tf_to_map)
        self._last_tag_map = pt_map

        # Transform the tag's rotation into map frame to get orientation quaternion
        R_map = self._rot_from_tf(tf_to_map) @ R_tag
        self._last_tag_quat = self._rot_to_quat(R_map)

        # Compute table centre in map frame:
        # table_centre = tag_position − normal_map * (table_side / 2)
        # (subtract along normal because the normal points TOWARD the robot,
        #  so centre is BEHIND the tag face by half the table depth)
        table_side = float(self.get_parameter('table_side').value)
        table_top  = float(self.get_parameter('table_top_z').value)
        n_map = R_map[:, 2]   # outward normal in map frame (toward robot)
        drop_map = PointStamped()
        drop_map.header.frame_id = 'map'
        drop_map.header.stamp    = pt_map.header.stamp
        drop_map.point.x = pt_map.point.x - float(n_map[0]) * (table_side / 2.0)
        drop_map.point.y = pt_map.point.y - float(n_map[1]) * (table_side / 2.0)
        # z: table top converted to base_link will be done in publish; store map world z
        drop_map.point.z = table_top   # world z (constant; arm z computed per level)
        self._last_drop_map = drop_map

        # Immediate publish + log
        self._publish_to_base_link(pt_map, drop_map, R_map, tag_id)
        self.get_logger().info(
            f'Tag {tag_id} detected: '
            f'tag@map ({pt_map.point.x:.3f},{pt_map.point.y:.3f}), '
            f'drop@map ({drop_map.point.x:.3f},{drop_map.point.y:.3f})',
            throttle_duration_sec=1.0)

    # ── Publish to base_link ──────────────────────────────────────────────────

    def _publish_to_base_link(self, pt_map, drop_map, R_map, tag_id):
        try:
            tf_to_base = self._tf_buf.lookup_transform(
                TARGET_FRAME, 'map',
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except Exception:
            return

        pt_base   = do_transform_point(pt_map,  tf_to_base)
        drop_base = do_transform_point(drop_map, tf_to_base)

        # Tag pose (full 6-DOF) in base_link
        qx, qy, qz, qw = self._last_tag_quat
        tag_ps = PoseStamped()
        tag_ps.header.frame_id = TARGET_FRAME
        tag_ps.header.stamp    = self.get_clock().now().to_msg()
        tag_ps.pose.position   = pt_base.point
        tag_ps.pose.orientation.x = qx
        tag_ps.pose.orientation.y = qy
        tag_ps.pose.orientation.z = qz
        tag_ps.pose.orientation.w = qw
        self._tag_pub.publish(tag_ps)

        # Drop point in base_link
        drop_ps = PointStamped()
        drop_ps.header.frame_id = TARGET_FRAME
        drop_ps.header.stamp    = self.get_clock().now().to_msg()
        drop_ps.point = drop_base.point
        self._drop_pub.publish(drop_ps)

        self._publish_markers(tag_ps, drop_ps, tag_id)

    def _publish_latched(self):
        """Republish the last known detection at 20 Hz, dynamically in base_link."""
        if self._last_tag_map is None or self._last_drop_map is None:
            return
        try:
            tf = self._tf_buf.lookup_transform(
                TARGET_FRAME, 'map',
                rclpy.time.Time(), timeout=Duration(seconds=0.05))
        except Exception:
            return

        pt_base   = do_transform_point(self._last_tag_map,  tf)
        drop_base = do_transform_point(self._last_drop_map, tf)

        qx, qy, qz, qw = self._last_tag_quat
        tag_ps = PoseStamped()
        tag_ps.header.frame_id = TARGET_FRAME
        tag_ps.header.stamp    = self.get_clock().now().to_msg()
        tag_ps.pose.position   = pt_base.point
        tag_ps.pose.orientation.x = qx
        tag_ps.pose.orientation.y = qy
        tag_ps.pose.orientation.z = qz
        tag_ps.pose.orientation.w = qw
        self._tag_pub.publish(tag_ps)

        drop_ps = PointStamped()
        drop_ps.header.frame_id = TARGET_FRAME
        drop_ps.header.stamp    = self.get_clock().now().to_msg()
        drop_ps.point = drop_base.point
        self._drop_pub.publish(drop_ps)

    # ── Markers for RViz ──────────────────────────────────────────────────────

    def _publish_markers(self, tag_ps: PoseStamped, drop_ps: PointStamped, tag_id: int):
        # Tag face: cyan sphere
        m = Marker()
        m.header = tag_ps.header
        m.ns, m.id   = 'place_tag', tag_id
        m.type, m.action = Marker.SPHERE, Marker.ADD
        m.pose = tag_ps.pose
        m.scale.x = m.scale.y = m.scale.z = 0.04
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 1.0, 0.9
        self._mark_pub.publish(m)

        # Drop point: yellow sphere
        d = Marker()
        d.header.frame_id = TARGET_FRAME
        d.header.stamp    = tag_ps.header.stamp
        d.ns, d.id   = 'place_drop', 0
        d.type, d.action = Marker.SPHERE, Marker.ADD
        d.pose.position  = drop_ps.point
        d.pose.orientation.w = 1.0
        d.scale.x = d.scale.y = d.scale.z = 0.05
        d.color.r, d.color.g, d.color.b, d.color.a = 1.0, 1.0, 0.0, 0.9
        self._mark_pub.publish(d)

    # ── Math helpers ──────────────────────────────────────────────────────────

    @staticmethod
    def _rot_from_tf(tf_stamped):
        """Extract 3×3 rotation matrix from a TransformStamped."""
        q = tf_stamped.transform.rotation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
        ])
        return R

    @staticmethod
    def _rot_to_quat(R):
        """3×3 rotation matrix → (qx, qy, qz, qw), Shepperd method."""
        tr = R[0, 0] + R[1, 1] + R[2, 2]
        if tr > 0:
            s = 0.5 / math.sqrt(tr + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return float(x), float(y), float(z), float(w)


def main(args=None):
    rclpy.init(args=args)
    node = TagDockEstimator()
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
