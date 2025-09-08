#!/usr/bin/env python3

# Import the necessary libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DisplaySubscriber(Node):
    """
    A ROS2 Node that subscribes to an image topic and displays the image using OpenCV.
    This is the Consumer in our system.
    """

    def __init__(self):
        """
        Constructor for the DisplaySubscriber node.
        Initializes the node, sets up the CV bridge, and creates the subscription.
        """
        # Initialize the parent Node class with the name 'display_subscriber'
        super().__init__('display_subscriber')
        
        # Instantiate the CvBridge object. This is our tool for converting ROS images to OpenCV images.
        self.bridge = CvBridge()
        
        # Create a Subscription.
        # - The first parameter (Image) specifies the type of message to listen for.
        # - The second parameter ('/camera/image_raw') is the name of the topic to subscribe to.
        #   This MUST match the topic name the camera_publisher is using.
        # - The third parameter (self.listener_callback) is the callback function that will be
        #   executed every time a new message is received on the topic.
        # - The fourth parameter (10) is the queue size. It tells ROS to buffer up to 10 messages
        #   in case the callback function is busy processing a previous one. If the queue fills up,
        #   old messages will be dropped.
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        # Prevent the subscription object from being destroyed prematurely by keeping an unused reference.
        self.subscription
        
        # Print a confirmation message to the terminal.
        self.get_logger().info('Display subscriber node has been started and is listening on /camera/image_raw...')

    def listener_callback(self, msg):
        """
        Callback function that is triggered whenever a new image message is received.
        
        Args:
            msg (sensor_msgs.msg.Image): The incoming image message.
        """
        try:
            # Convert the ROS Image message to an OpenCV image.
            # - 'msg' is the incoming ROS message.
            # - 'bgr8' is the desired encoding. This matches the 'bgr8' encoding we used in the publisher.
            #   OpenCV uses BGR color order by default.
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            # If the conversion fails for any reason, log the error and exit the callback.
            self.get_logger().error('Failed to convert image: %s' % e)
            return

        # Display the converted image in a window named 'Camera Feed'
        cv2.imshow('Camera Feed', cv_image)
        
        # Wait for 1 millisecond for a key press. This is necessary for cv2.imshow() to work.
        # The '& 0xFF' is a bitwise operation to get the last byte, which is standard for checking key presses.
        key = cv2.waitKey(1) & 0xFF
        
        # If the 'q' key is pressed, exit the node gracefully.
        if key == ord('q'):
            self.get_logger().info('Shutting down display subscriber node.')
            rclpy.shutdown()

def main(args=None):
    """
    Main function to initialize and run the ROS node.
    """
    # Initialize the ROS client library.
    rclpy.init(args=args)
    
    # Create an instance of the DisplaySubscriber node.
    display_subscriber = DisplaySubscriber()
    
    # Keep the node running and processing callbacks until it is explicitly shut down.
    try:
        rclpy.spin(display_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup: Destroy the node and shutdown ROS.
        display_subscriber.destroy_node()
        rclpy.shutdown()
        # Close all OpenCV windows that were created.
        cv2.destroyAllWindows()

# This standard Python idiom ensures the main() function runs only when the script is executed directly.
if __name__ == '__main__':
    main()