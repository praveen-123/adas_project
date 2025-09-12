#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YOLODetector(Node):
    """
    A ROS2 node that performs real-time object detection using YOLOv8 on images
    received from a camera topic.
    """
    def __init__(self):
        super().__init__('yolo_detector')
        self.get_logger().info("Initializing YOLO Detector node...")
        
        # Load the YOLOv8 model (using 'yolov8n.pt' for nano, fast version)
        self.model = YOLO('yolov8n.pt')  # You can change to 'yolov8s.pt' or 'yolov8m.pt' for better accuracy
        self.get_logger().info("YOLOv8 model loaded successfully.")
        
        # Initialize CV Bridge for ROS2-OpenCV conversion
        self.bridge = CvBridge()
        
        # Subscribe to the raw camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Topic from your camera_publisher node
            self.image_callback,  # Function to call when a message is received
            10  # Queue size
        )
        self.subscription  # Prevent unused variable warning
        
        # Create a publisher to output the detections
        self.detection_pub = self.create_publisher(Detection2DArray, '/yolo_detections', 10)
        
        self.get_logger().info("YOLO Detector node is ready. Waiting for images...")

    def image_callback(self, msg):
        """
        Callback function for the image subscriber.
        """
        try:
            # Convert the ROS Image message to an OpenCV image (BGR format)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # Run YOLOv8 inference on the image
        results = self.model(cv_image)
        
        # Create a Detection2DArray message to hold all detections
        detection_array = Detection2DArray()
        detection_array.header = msg.header  # Use the same timestamp as the input image

        # Parse the results from YOLO
        for result in results:
            for box in result.boxes:
                # Extract bounding box coordinates (x1, y1, x2, y2) in pixel coordinates
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                # Get the confidence score
                confidence = box.conf[0].item()
                # Get the class ID and convert it to the class name (e.g., 0 -> 'person')
                class_id = int(box.cls[0].item())
                class_name = result.names[class_id]

                # Create a single Detection2D message for this object
                detection = Detection2D()
                
                # Define the bounding box in terms of its center and size
                detection.bbox.center.position.x = (x1 + x2) / 2.0  # Center x coordinate
                detection.bbox.center.position.y = (y1 + y2) / 2.0  # Center y coordinate
                detection.bbox.size_x = x2 - x1  # Width of the box
                detection.bbox.size_y = y2 - y1  # Height of the box

                # Create a hypothesis for the classification result
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = class_name  # The class name (e.g., 'car')
                hypothesis.hypothesis.score = confidence     # The confidence score (0.0 to 1.0)
                detection.results.append(hypothesis)

                # Add this detection to the array
                detection_array.detections.append(detection)

        # Publish the complete array of detections
        self.detection_pub.publish(detection_array)
        # Log the number of detections (throttled to avoid spamming the console)
        self.get_logger().info(f"Published {len(detection_array.detections)} detection(s).", throttle_duration_sec=1.0)

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