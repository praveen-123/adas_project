#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')
        
        # Path to KITTI images (using the left color camera - image_02)
        self.image_folder = '/home/pravinhiremath/adas_project/simulations/2011_09_26_drive_0005_extract/2011_09_26/2011_09_26_drive_0005_extract/image_02/data'
        
        # Get list of all PNG files and sort them
        self.files = sorted([os.path.join(self.image_folder, f) for f in os.listdir(self.image_folder) if f.endswith('.png')])
        
        if not self.files:
            self.get_logger().error(f"No images found in {self.image_folder}")
            raise RuntimeError("No images found.")
            
        self.index = 0
        self.get_logger().info(f"Found {len(self.files)} images in {self.image_folder}")

        # Create publisher
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        # Create a timer to publish images at 10 Hz (KITTI was recorded at 10 Hz)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Read the next image in sequence
        img_path = self.files[self.index]
        frame = cv2.imread(img_path)
        
        if frame is None:
            self.get_logger().error(f"Failed to read image: {img_path}")
            return
            
        # KITTI images are high resolution (1242x375), let's resize to a more manageable size
        frame_resized = cv2.resize(frame, (640, 360))  # Maintain aspect ratio
        
        # Convert to ROS message and publish
        try:
            msg = self.bridge.cv2_to_imgmsg(frame_resized, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "kitti_camera"
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published: {os.path.basename(img_path)}', throttle_duration_sec=1.0)
        except Exception as e:
            self.get_logger().error(f"Error publishing image: {e}")
            
        # Move to next image, loop back to start when we reach the end
        self.index = (self.index + 1) % len(self.files)

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()