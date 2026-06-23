#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class BoxFollowerNode(Node):
    def __init__(self):
        super().__init__('box_follower')
        self.bridge = CvBridge()
        self.latest_depth_msg = None
        self.target_reached = False
        
        # Subscriptions
        self.rgb_sub = self.create_subscription(
            Image,
            '/rgb/image_raw',
            self.rgb_callback,
            10
        )
        self.depth_sub = self.create_subscription(
            Image,
            '/depth_camera/depth/image_raw',
            self.depth_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/perception/box_follower_debug', 10)
        
        self.get_logger().info("Box Follower Node Initialized. Waiting for sensor data...")

    def depth_callback(self, msg):
        self.latest_depth_msg = msg

    def rgb_callback(self, msg):
        if self.latest_depth_msg is None:
            self.get_logger().info("RGB received but waiting for first depth frame...", throttle_duration_sec=2.0)
            return

        try:
            # Convert ROS images to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_depth = self.bridge.imgmsg_to_cv2(self.latest_depth_msg, "32FC1")
        except Exception as e:
            self.get_logger().error(f"Failed to convert images: {e}")
            return

        h, w = cv_image.shape[:2]
        center_x = w / 2.0

        # Convert to HSV and segment purely vivid "Gazebo Blue"
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([110, 150, 50])
        upper_blue = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Morphological operations to clean up noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        box_detected = False
        twist = Twist()

        if contours:
            # Debug: log the sizes of all detected blue objects
            areas = [cv2.contourArea(c) for c in contours]
            self.get_logger().info(f"Detected contours: {areas}", throttle_duration_sec=2.0)

            # Query depth for each contour and select the one that is closest to the robot
            candidates = []
            for c in contours:
                if cv2.contourArea(c) < 30:  # Ignore tiny noise
                    continue
                M = cv2.moments(c)
                if M["m00"] == 0:
                    continue
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # Query depth around centroid
                depths = []
                half_w = 2
                for dy in range(-half_w, half_w + 1):
                    for dx in range(-half_w, half_w + 1):
                        ny, nx = cY + dy, cX + dx
                        if 0 <= ny < h and 0 <= nx < w:
                            d = cv_depth[ny, nx]
                            if not np.isnan(d) and not np.isinf(d) and d > 0.05:
                                depths.append(d)

                if depths:
                    distance = np.median(depths)
                    candidates.append((distance, c, cX, cY))

            if candidates:
                # Sort by distance (closest first)
                candidates.sort(key=lambda x: x[0])
                distance, largest_contour, cX, cY = candidates[0]
                box_detected = True

                # Target pick distance: 0.11m — camera is 10cm ahead of arm base,
                # so 0.11m from camera = 0.21m from arm base = within arm's reach (max 0.22m)
                TARGET_DISTANCE = 0.11

                dist_err = distance - TARGET_DISTANCE
                heading_err = (center_x - cX) / center_x

                # Visual feed annotations
                cv2.drawContours(cv_image, [largest_contour], -1, (0, 255, 0), 2)
                cv2.circle(cv_image, (cX, cY), 7, (0, 0, 255), -1)
                cv2.putText(cv_image, f"Dist: {distance:.3f}m | H_Err: {heading_err:.2f}",
                            (cX - 80, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

                # Control Loop logic
                if dist_err > 0.025:  # Tolerance threshold of 2.5 cm
                    # Proportional drive with limits (prevents slip/stiction)
                    twist.linear.x = float(np.clip(dist_err * 0.6, 0.06, 0.25))
                    twist.angular.z = float(heading_err * 1.0)
                    self.target_reached = False
                    self.get_logger().info(
                        f"Approaching box... Dist: {distance:.3f}m, Speed: {twist.linear.x:.2f}, Steer: {twist.angular.z:.2f}",
                        throttle_duration_sec=1.0
                    )
                else:
                    # Within picking range — stop completely and hold
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    if not self.target_reached:
                        self.get_logger().info(f"Target reached! Stopped at {distance:.3f}m from box. Standing by.")
                        self.target_reached = True
            
        if not box_detected:
            self.get_logger().info("Scanning... No blue box visible.", throttle_duration_sec=2.0)
            # Stop or crawl slowly in search
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Publish velocity
        self.cmd_vel_pub.publish(twist)

        # Publish debug overlay
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish debug image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BoxFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop base on exit
        stop_twist = Twist()
        node.cmd_vel_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
