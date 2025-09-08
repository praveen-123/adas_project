#!/usr/bin/env python3

# Import necessary libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import sys

class CameraPublisher(Node):
    """
    A ROS2 Node that publishes camera frames as sensor_msgs/Image messages.
    This is the Producer in our system.
    """

    def __init__(self, image_folder=None, fps=10):
        """
        Constructor for the CameraPublisher node.
        
        Args:
            image_folder (str, optional): Path to a folder of images. If None, uses webcam.
            fps (int): Frames per second for the publishing rate.
        """
        # Initialize the parent Node class. Must call this.
        # The name 'camera_publisher' will appear in ROS2 logs and tools.
        super().__init__('camera_publisher')
        
        # Create a Publisher object.
        # QOS Profile (Quality of Service): This defines the "reliability" of the communication.
        #   History: KEEP_LAST - Keep only the last N messages.
        #   Depth: 10 - Keep 10 messages in a buffer in case a subscriber is slow.
        #   Reliability: RELIABLE - Guarantee that messages will be delivered.
        #   This is a common profile for critical sensor data.
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # Create a timer. This will call the `timer_callback` method every 1/fps seconds.
        timer_period = 1.0 / fps  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Create an instance of the CvBridge class for image conversion.
        self.bridge = CvBridge()
        
        # Store the path to the image folder and get a sorted list of image files.
        self.image_folder = image_folder
        if self.image_folder:
            # Check if the provided path exists and is a directory.
            if not os.path.isdir(self.image_folder):
                self.get_logger().error(f"Directory {image_folder} does not exist.")
                sys.exit(1)
            # List and sort all files in the directory. A simple filter for image files.
            self.image_files = sorted([os.path.join(self.image_folder, f) for f in os.listdir(self.image_folder) 
                                       if f.endswith(('.png', '.jpg', '.jpeg'))])
            self.get_logger().info(f"Found {len(self.image_files)} images in {image_folder}")
            self.frame_index = 0
        else:
            self.get_logger().info("No image folder provided. Attempting to use webcam...")
            # Initialize the webcam. 0 is usually the default built-in camera.
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                self.get_logger().error("Cannot open webcam")
                sys.exit(1)

    def timer_callback(self):
        """
        Callback function called by the timer at a fixed interval.
        This is where the work of grabbing a frame and publishing it happens.
        """
        # 1. ACQUIRE THE FRAME
        if self.image_folder:
            # Read the next image from the list of files
            image_path = self.image_files[self.frame_index]
            cv_image = cv2.imread(image_path)
            if cv_image is None:
                self.get_logger().error(f"Could not read image: {image_path}")
                return
            # Cycle through the image list
            self.frame_index = (self.frame_index + 1) % len(self.image_files)
        else:
            # Read a frame from the webcam
            ret, cv_image = self.cap.read()
            if not ret:
                self.get_logger().error("Failed to grab frame from webcam")
                return

        # 2. (Optional) PROCESS THE FRAME - Let's resize for consistency.
        # This ensures all images are the same size, which is crucial for downstream processing.
        cv_image = cv2.resize(cv_image, (640, 480))

        # 3. CONVERT OpenCV image -> ROS2 Image message
        try:
            # The `cv2_to_imgmsg` method does the conversion.
            # Encoding 'bgr8' is standard for color images in OpenCV.
            ros_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        except Exception as e:
            # Always handle potential conversion errors gracefully.
            self.get_logger().error(f"CV Bridge conversion failed: {e}")
            return

        # 4. ADD A TIMESTAMP TO THE MESSAGE HEADER
        # This is VITALLY important. Every sensor message MUST have an accurate timestamp.
        # It allows other nodes to synchronize data from different sensors (e.g., camera and LiDAR).
        ros_image_msg.header.stamp = self.get_clock().now().to_msg()
        # You can also set the frame_id for advanced transformation trees (for later).
        ros_image_msg.header.frame_id = 'camera_link'

        # 5. PUBLISH THE MESSAGE
        self.publisher_.publish(ros_image_msg)
        
        # 6. LOG THE ACTION (Use DEBUG level to avoid spamming the console in a real system)
        self.get_logger().debug(f"Published image message with timestamp {ros_image_msg.header.stamp.sec}.{ros_image_msg.header.stamp.nanosec}")

def main(args=None):
    """
    Main function which is the entry point for the node.
    """
    # Initialize the ROS2 client library. Must be called first.
    rclpy.init(args=args)
    
    # Create an instance of our CameraPublisher node.
    # Point it to the folder of images we generated on Day 1.
    image_folder_path = os.path.expanduser('~/adas_project/simulations')
    camera_publisher = CameraPublisher(image_folder=image_folder_path, fps=2) # 2 FPS is fine for testing
    
    # `rclpy.spin()` keeps the node alive so it can process callbacks (like our timer).
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        # Handle a graceful shutdown when user presses Ctrl+C
        pass
    finally:
        # Cleanup: Destroy the node explicitly and shutdown ROS.
        camera_publisher.destroy_node()
        rclpy.shutdown()

# This standard Python idiom ensures the main() function runs
# only when this script is executed directly.
if __name__ == '__main__':
    main()