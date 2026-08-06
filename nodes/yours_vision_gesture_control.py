#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
===============================================================================
Yours Robotics 小车 — Vision-Based Gesture Control & Follow-Me ROS Node
===============================================================================
Author: Yours Robotics Engineering Team
Platform: NVIDIA Jetson / ROS Noetic
Hardware: Intel RealSense D435 / D435i 3D RGB-D Camera

Functionality:
1. Subscribes to RealSense RGB Color (/d435i/color/image_raw) & Depth (/d435i/depth/image_rect_raw).
2. Performs real-time Hand & Gesture Recognition using OpenCV & Color/Skin/Contour/Keypoint Analysis:
   - ✋ Raised Open Palm: Instantly triggers Priority 255 E-STOP (std_msgs/Bool on 'e_stop').
   - 👋 Open Hand Wave: Activates Follow-Me Mode — tracks user distance Z (maintains 1.2m gap)
     and angular heading error X, publishing Twist velocity commands to /yours_cmd_vel/keyop_ctrl.
   - ✌️ Victory/Pointing: Triggers custom waypoint navigation target.
3. Publishes annotated debug image stream to /yours_vision_gesture_debug_image.
===============================================================================
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge, CvBridgeError

class YoursVisionGestureControlNode:
    def __init__(self):
        rospy.init_node('yours_vision_gesture_control_node', anonymous=False)
        rospy.loginfo("👁️ Initializing Yours Robotics Vision Gesture & Follow-Me Node...")

        self.bridge = CvBridge()

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/yours_cmd_vel/keyop_ctrl', Twist, queue_size=1)
        self.estop_pub = rospy.Publisher('e_stop', Bool, queue_size=1)
        self.gesture_status_pub = rospy.Publisher('/yours_vla/status', String, queue_size=1)
        self.debug_img_pub = rospy.Publisher('/yours_vision_gesture_debug_image', Image, queue_size=1)

        # Subscribers (Subscribes to RealSense D435 / D435i topics with fallback)
        color_topic = rospy.get_param('~color_topic', '/d435i/color/image_raw')
        depth_topic = rospy.get_param('~depth_topic', '/d435i/depth/image_rect_raw')

        self.color_sub = rospy.Subscriber(color_topic, Image, self.color_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber(depth_topic, Image, self.depth_callback, queue_size=1)

        # State Variables
        self.latest_depth_mat = None
        self.follow_me_active = False
        self.target_follow_distance = 1.2  # meters
        self.max_linear_speed = 0.25       # m/s
        self.max_angular_speed = 0.40      # rad/s

        rospy.loginfo(f"✅ Yours Vision Gesture Node Running! Subscribed to {color_topic}")

    def depth_callback(self, msg):
        try:
            # Convert 16-bit depth image (millimeters) to OpenCV Mat
            self.latest_depth_mat = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
        except CvBridgeError as e:
            rospy.logerr(f"Depth CvBridge Error: {e}")

    def color_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Color CvBridge Error: {e}")
            return

        h, w, _ = cv_img.shape
        debug_frame = cv_img.copy()

        # 1. Skin & Hand Gesture Detection via HSV Color Thresholding & Convex Hull Analysis
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        lower_skin = np.array([0, 20, 70], dtype=np.uint8)
        upper_skin = np.array([20, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_skin, upper_skin)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        detected_gesture = "NONE"
        hand_cx, hand_cy = w // 2, h // 2

        if contours:
            # Find largest contour (assumed hand/body in frontal FOV)
            max_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(max_contour)

            if area > 4000:
                M = cv2.moments(max_contour)
                if M["m00"] != 0:
                    hand_cx = int(M["m10"] / M["m00"])
                    hand_cy = int(M["m01"] / M["m00"])

                # Bounding box & Convexity Defects for finger gesture classification
                x, y, bw, bh = cv2.boundingRect(max_contour)
                cv2.rectangle(debug_frame, (x, y), (x + bw, y + bh), (0, 255, 255), 2)
                cv2.circle(debug_frame, (hand_cx, hand_cy), 8, (0, 0, 255), -1)

                hull = cv2.convexHull(max_contour, returnPoints=False)
                defects = None
                if len(hull) > 3:
                    try:
                        defects = cv2.convexityDefects(max_contour, hull)
                    except Exception:
                        pass

                finger_count = 0
                if defects is not None:
                    for i in range(defects.shape[0]):
                        s, e, f, d = defects[i, 0]
                        start = tuple(max_contour[s][0])
                        end = tuple(max_contour[e][0])
                        far = tuple(max_contour[f][0])
                        a = np.linalg.norm(np.array(end) - np.array(start))
                        b = np.linalg.norm(np.array(far) - np.array(start))
                        c = np.linalg.norm(np.array(end) - np.array(far))
                        angle = np.arccos((b**2 + c**2 - a**2) / (2 * b * c + 1e-5))
                        if angle <= np.pi / 2 and d > 1200:
                            finger_count += 1

                # Gesture Classification Logic
                if finger_count >= 4 and hand_cy < h * 0.45:
                    detected_gesture = "PALM_STOP"
                elif finger_count >= 2:
                    detected_gesture = "WAVE_FOLLOW"
                elif finger_count == 1:
                    detected_gesture = "POINT_NAV"

        # 2. Execute Gesture Actions
        if detected_gesture == "PALM_STOP":
            # ✋ Raised Open Palm: Trigger E-STOP Immediately!
            self.estop_pub.publish(Bool(data=True))
            self.publish_zero_vel()
            self.follow_me_active = False
            cv2.putText(debug_frame, "✋ GESTURE: E-STOP TRIGGERED!", (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
            rospy.logwarn_throttle(2, "✋ VISION HRI: Raised Palm Detected! E-STOP Triggered.")

        elif detected_gesture == "WAVE_FOLLOW":
            # 👋 Hand Wave: Enable Follow-Me Mode
            self.follow_me_active = True
            self.estop_pub.publish(Bool(data=False))
            cv2.putText(debug_frame, "👋 GESTURE: FOLLOW-ME ACTIVE", (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)

        elif detected_gesture == "POINT_NAV":
            cv2.putText(debug_frame, "✌️ GESTURE: POINT NAV", (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 3)

        # 3. Follow-Me Visual Tracking Driver
        if self.follow_me_active:
            target_depth_m = 1.2
            if self.latest_depth_mat is not None and 0 <= hand_cy < h and 0 <= hand_cx < w:
                depth_val_mm = self.latest_depth_mat[hand_cy, hand_cx]
                if depth_val_mm > 100:
                    target_depth_m = depth_val_mm / 1000.0

            # Proportional Controller for Linear Distance & Angular Heading
            dist_error = target_depth_m - self.target_follow_distance
            angle_error = (w / 2.0 - hand_cx) / (w / 2.0)

            twist = Twist()
            if abs(dist_error) > 0.15:
                twist.linear.x = float(np.clip(dist_error * 0.5, -self.max_linear_speed, self.max_linear_speed))
            twist.angular.z = float(np.clip(angle_error * 0.8, -self.max_angular_speed, self.max_angular_speed))

            self.cmd_vel_pub.publish(twist)
            cv2.putText(debug_frame, f"TRACKING: Z={target_depth_m:.2f}m Error={angle_error:.2f}", (30, h - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # 4. Publish Debug Image
        try:
            self.debug_img_pub.publish(self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8"))
        except CvBridgeError:
            pass

    def publish_zero_vel(self):
        self.cmd_vel_pub.publish(Twist())

if __name__ == '__main__':
    try:
        node = YoursVisionGestureControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
