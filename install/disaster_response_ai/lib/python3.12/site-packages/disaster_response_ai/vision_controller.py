#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

# PX4 specific messages (Need to be built or mapped, simplified here for simulation logic)
# Standard ROS 2 approach allows us to use standard topics if we bridge them.

from ultralytics import YOLO

class DisasterDroid(Node):
    def __init__(self):
        super().__init__('ai_vision_controller')

        # --- QoS Profile for PX4 (Best Effort is required for sensor data) ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- MODEL SETUP ---
        self.get_logger().info("Loading YOLOv8 & Logic...")
        self.model_survivor = YOLO("yolov8n.pt") 

        # --- SUBSCRIBERS ---
        self.bridge = CvBridge()
        # Note: Gazebo Harmonic topic names might differ slightly, check via 'ros2 topic list'
        self.create_subscription(Image, '/camera/image_raw', self.rgb_callback, qos_profile)
        self.create_subscription(Image, '/camera/depth_image', self.depth_callback, qos_profile)

        # --- PUBLISHERS (Commanding the Drone) ---
        # In ROS 2 PX4, we typically publish to /fmu/in/trajectory_setpoint
        # For simplicity in this demo, we print the logic commands
        self.get_logger().info("AI Controller Active")

    def rgb_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # --- MODEL 1: Survivor (YOLO) ---
            results = self.model_survivor(cv_image, verbose=False)
            
            # --- MODEL 2: Fire (HSV Proxy) ---
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array([18, 50, 50]), np.array([35, 255, 255]))
            fire_detected = cv2.countNonZero(mask) > 500

            # --- DECISION LOGIC ---
            if fire_detected:
                self.get_logger().warn("🔥 FIRE DETECTED! HOLDING POSITION.")
            else:
                # If deployment, publish offboard setpoints here
                pass

            # Visualization
            annotated_frame = results[0].plot()
            if fire_detected:
                cv2.putText(annotated_frame, "FIRE", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            cv2.imshow("Jazzy AI View", annotated_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Frame Error: {e}")

    def depth_callback(self, msg):
        # --- MODEL 3: Depth/Safe Landing ---
        cv_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        if np.nanmean(cv_depth) < 1.0:
            self.get_logger().info("⚠️ Terrain too close!")

def main(args=None):
    rclpy.init(args=args)
    node = DisasterDroid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
