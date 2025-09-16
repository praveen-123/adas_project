#!/usr/bin/env python3

import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import os
import time
from collections import Counter

class YOLODetector(Node):
    """
    A ROS2 node that performs real-time object detection using YOLOv8n and YOLOv8s
    on images received from a camera topic. Logs FPS, mean confidence, and class counts.
    """
    def __init__(self):
        super().__init__('yolo_detector')
        self.get_logger().info("Initializing YOLO Detector node...")

        # Load YOLO models
        #comment out:
        #self.model_n = YOLO('yolov8n.pt')
        #keep:
        self.model_s = YOLO('yolov8s.pt')
        # comment out:
        #self.get_logger().info("YOLOv8n model loaded successfully.")
        #keep:
        self.get_logger().info("YOLOv8s model loaded successfully.")

        # Detect device (GPU if available, else CPU)
        self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        #comment out:
        #self.model_n.to(self.device)
        #keep:
        self.model_s.to(self.device)
        self.get_logger().info(f"Using device: {self.device}")

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher (we use YOLOv8s results for ROS messages)
        self.detection_pub = self.create_publisher(Detection2DArray, '/yolo_detections', 10)

        # Output folder
        self.output_dir = "/home/pravinhiremath/adas_project/output_frames"
        os.makedirs(self.output_dir, exist_ok=True)

        self.prev_time = time.time()
        self.get_logger().info("YOLO Detector node is ready. Waiting for images...")

    def log_results(self, model_name, results, fps):
        """Log FPS, mean confidence, and class counts for a given model."""
        confidences = []
        classes = []
        for result in results:
            for box in result.boxes:
                confidences.append(box.conf[0].item())
                classes.append(result.names[int(box.cls[0].item())])

        mean_conf = sum(confidences) / len(confidences) if confidences else 0.0
        counts = dict(Counter(classes))

        self.get_logger().info(
            f"[{model_name}] FPS: {fps:.2f} | Mean Conf: {mean_conf:.2f} | Counts: {counts}"
        )

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # Calculate FPS
        current_time = time.time()
        fps = 1.0 / (current_time - self.prev_time)
        self.prev_time = current_time

        # Run inference with both models
        #comment out:
        #results_n = self.model_n(cv_image, device=self.device, verbose=False)
        #keep:
        results_s = self.model_s(cv_image, device=self.device, verbose=False)

        # Log separately
        #comment out:
        #self.log_results("YOLOv8n", results_n, fps)
        #keep:
        self.log_results("YOLOv8s", results_s, fps)

        # Save YOLOv8s result frame (better accuracy)
        #comment out:
        # output_path = os.path.join(self.output_dir, f"yolov8n_frame_{msg.header.stamp.sec}_{msg.header.stamp.nanosec}.jpg")
        # cv2.imwrite(output_path, results_n[0].plot())

        #keep:
        output_path = os.path.join(self.output_dir, f"yolov8s_frame_{msg.header.stamp.sec}_{msg.header.stamp.nanosec}.jpg")
        cv2.imwrite(output_path, results_s[0].plot())

        # Publish detections (from YOLOv8s)
        detection_array = Detection2DArray()
        detection_array.header = msg.header
        #comment out:
        #for result in results_n:
        #keep:
        for result in results_s:
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                confidence = box.conf[0].item()
                class_id = int(box.cls[0].item())
                class_name = result.names[class_id]

                detection = Detection2D()
                detection.bbox.center.position.x = (x1 + x2) / 2.0
                detection.bbox.center.position.y = (y1 + y2) / 2.0
                detection.bbox.size_x = x2 - x1
                detection.bbox.size_y = y2 - y1

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = class_name
                hypothesis.hypothesis.score = confidence
                detection.results.append(hypothesis)

                detection_array.detections.append(detection)

        self.detection_pub.publish(detection_array)

def main(args=None):
    rclpy.init(args=args)
    node = YOLODetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("YOLO Detector node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
