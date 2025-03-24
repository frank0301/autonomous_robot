#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image  # Import Image message type from sensor_msgs
import cv2  # OpenCV library for image processing
from cv_bridge import CvBridge  # Bridge between ROS and OpenCV

#The purpose of this code is to create a ROS 2 node that captures RGB images from a USB
#  camera and publishes them as Image messages on a specified topic.

class CameraRGBPublisher(Node):
    """A ROS 2 node that publishes RGB camera images."""

    def __init__(self):
        """Initialize the CameraRGBPublisher node."""
        # Call the parent class constructor to initialize the node
        super().__init__("camera_rgb_publisher")

        # Create a publisher to publish Image messages on the 'camera_rgb/image_raw' topic
        self.pub_ = self.create_publisher(Image, 'camera_rgb/image_raw', 10)

        # Set up a timer to call the timer_callback function every 0.1 seconds
        self.timer_ = self.create_timer(0.1, self.timer_callback)

        # Open a connection to the USB camera (use the appropriate index for your camera)
        self.cap = cv2.VideoCapture(2)  # For USB camera
        # Note: Ensure OpenCV (cv2) is installed on the microcontroller

        # Initialize CvBridge to convert between OpenCV images and ROS Image messages
        self.cv_bridge = CvBridge()

    def timer_callback(self):
        """Callback function to capture and publish camera images."""
        # Read a frame from the camera
        ret, frame = self.cap.read()
        if ret:
            # Convert the captured frame (OpenCV format) to a ROS Image message
            msg = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")

            # Log that the camera data is being published
            self.get_logger().info("Publishing camera data...")
            # Publish the message to the 'camera_rgb/image_raw' topic
            self.pub_.publish(msg)
        else:
            # Log an error message if the image capture fails
            self.get_logger().error("Failed to capture image")

    def destroy_node(self):
        """Release the camera resource and destroy the node."""
        # Release the camera when the node is destroyed
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    # Create an instance of the CameraRGBPublisher node
    node = CameraRGBPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__== "__main__":
    main() 
