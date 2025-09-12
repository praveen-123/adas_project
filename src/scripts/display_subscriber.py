#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import cv2

class DisplaySubscriber(Node):
    """
    A ROS2 node that subscribes to both an image topic and a detection topic.
    It visualizes the detections by drawing bounding boxes on the image.
    """
    def __init__(self):
        super().__init__('display_subscriber')
        self.bridge = CvBridge()
        
        # We need the latest image and the latest detections to draw on it
        self.current_image = None
        self.current_detections = None
        
        # Subscribe to the raw camera image
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        # Subscribe to the YOLO detections
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/yolo_detections',
            self.detection_callback,
            10
        )
        
        self.get_logger().info("Display subscriber node started. Waiting for data...")

    def image_callback(self, msg):
        """Store the latest image."""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")

    def detection_callback(self, msg):
        """Store the latest detections and trigger processing."""
        self.current_detections = msg
        self.process_and_display()

    def process_and_display(self):
        """If we have both an image and detections, draw the boxes and display."""
        if self.current_image is None or self.current_detections is None:
            return # Wait until we have both messages

        # Create a copy of the image to draw on
        img_to_show = self.current_image.copy()
        
        for detection in self.current_detections.detections:
            # Extract bounding box parameters from the message
            center_x = detection.bbox.center.position.x
            center_y = detection.bbox.center.position.y
            width = detection.bbox.size_x
            height = detection.bbox.size_y
            
            # Calculate top-left and bottom-right corners for OpenCV's rectangle function
            x1 = int(center_x - width / 2)
            y1 = int(center_y - height / 2)
            x2 = int(center_x + width / 2)
            y2 = int(center_y + height / 2)
            
            # Draw a green rectangle around the detected object
            color = (0, 255, 0)  # Green in BGR format
            thickness = 2
            cv2.rectangle(img_to_show, (x1, y1), (x2, y2), color, thickness)
            
            # Add a label with the class name and confidence score
            if detection.results: # Check if there is a classification result
                class_name = detection.results[0].hypothesis.class_id
                confidence = detection.results[0].hypothesis.score
                label = f"{class_name}: {confidence:.2f}" # Format: 'car: 0.92'
                
                # Draw the label background
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.6
                label_size = cv2.getTextSize(label, font, font_scale, thickness)[0]
                cv2.rectangle(img_to_show, (x1, y1 - label_size[1] - 10), (x1 + label_size[0], y1), color, -1) # Filled rectangle
                # Draw the label text
                cv2.putText(img_to_show, label, (x1, y1 - 5), font, font_scale, (0, 0, 0), thickness) # Black text
        
        # Display the final image with detections
        cv2.imshow("YOLO Object Detections", img_to_show)
        # Wait for 1 millisecond. This is necessary for OpenCV to update the window.
        # If 'q' is pressed, exit the program.
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("User requested to shut down.")
            raise KeyboardInterrupt # This will trigger the shutdown sequence

def main(args=None):
    rclpy.init(args=args)
    node = DisplaySubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Display node shut down successfully.")
    finally:
        # Ensure all OpenCV windows are closed when the node stops
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()