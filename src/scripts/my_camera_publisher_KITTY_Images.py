#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class cameraPublisher(Node):
    def __init__(self):
        super().__init__('cameraPublisher')

        self.image_folder = '/home/pravinhiremath/adas_project/simulations/2011_09_26_drive_0005_extract/2011_09_26/2011_09_26_drive_0005_extract/image_02/data'
        self.files = sorted([os.path.join(self.image_folder, f) for f in os.listdir(self.image_folder) if f.endswith('.png')])

        if not self.files:
            self.get_logger().error(f"Image not found in {self.image_folder}")
            raise RuntimeError("Image not found")
        
        self.index = 0

        self.get_logger().info(f"found {len(self.files)} images in {self.image_folder}")


        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        img_path = self.files[self.index]
        frame = cv2.imread(img_path)

        if frame is None:
            self.get_logger().error(f"failed to read Image : {img_path}")
            return
        
        frame_resized = cv2.resize(frame, (640, 360))

        try:
            msg = self.bridge.cv2_to_imgmsg(frame_resized, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'Kitty_Images'
            self.publisher_.publish(msg)
            self.get_logger().info(f"published: {os.path.basename(img_path)}", throttle_duration_sec = 1.0)

        except Exception as e:
            self.get_logger().error(f"error publishing image{e}")

        self.index = (self.index + 1) % len(self.files)

def main(args = None):
    rclpy.init(args=args)
    node = cameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
        
        
        
        
        
