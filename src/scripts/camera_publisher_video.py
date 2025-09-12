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
        
        # Create the publisher on the topic '/camera/image_raw'
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        
        # Get the absolute path to the test video
        project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        video_path = os.path.join(project_root, 'simulations', 'myvideo.mp4')
        self.get_logger().info(f"Using video path: {video_path}")
        
        # Open the video file
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open video file")
            raise RuntimeError("Could not open video file")
            
        self.get_logger().info("Camera publisher node started with video file.")

        # Create a timer that fires 30 times per second (30 Hz) to call the timer_callback function
        timer_period = 1/30  # seconds (30 FPS)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """
        This function is called periodically by the timer.
        It captures a frame and publishes it.
        """
        # Capture frame-by-frame
        ret, frame = self.cap.read()
        
        if ret == True:
            # Preprocessing: Resize the frame for consistency.
            frame_resized = cv2.resize(frame, (640, 480))
            
            # Convert the OpenCV image to a ROS 2 Image message
            ros_image_msg = self.bridge.cv2_to_imgmsg(frame_resized, encoding="bgr8")
            
            # Publish the image message to the topic
            self.publisher_.publish(ros_image_msg)
            
            # Log a message at a controlled rate to avoid spamming the console
            self.get_logger().info('Publishing a frame', throttle_duration_sec=2.0)
            
        else:
            # If we reach the end of the video, loop back to the beginning
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            self.get_logger().info('End of video. Looping...', throttle_duration_sec=1.0)

    def __del__(self):
        """Destructor to ensure the camera is released when the node is destroyed."""
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        self.get_logger().info("Camera released.")

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    
    try:
        # Keep the node running until it is explicitly shut down
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        # Handle a graceful shutdown when the user presses Ctrl+C
        camera_publisher.get_logger().info('Node stopped cleanly by user.')
    finally:
        # Destroy the node explicitly and shutdown ROS
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()