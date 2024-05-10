# node1.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class VideoPublisher(Node):
    """
    A ROS2 node to publish video frames from a camera as ROS Image messages.
    """
    def __init__(self):
        super().__init__('video_publisher_rp')
        # Initialize the publisher for '/rpi_video' topic with Image message type
        self.publisher_ = self.create_publisher(Image, '/rpi_video', 10)
        
        # Set the period (in seconds) between callback executions
        timer_period = 0.05  # 20 frames per second
        self.timer = self.create_timer(timer_period, self.camera_callback)
        
        # Open the default camera device
        self.cap = cv2.VideoCapture(0)
        # Set the capture properties, e.g., frame height and width
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        
        # Initialize the CvBridge, which converts between ROS Image messages and OpenCV images
        self.bridge = CvBridge()

    def camera_callback(self):
        # Capture a frame from the camera
        ret, frame = self.cap.read()
        if ret:
            # Convert the frame to grayscale
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # Convert the OpenCV image to a ROS Image message
            frame_msg = self.bridge.cv2_to_imgmsg(frame, 'mono8')
            # Publish the Image message
            self.publisher_.publish(frame_msg)
        else:
            # Log an error if no frame is captured
            self.get_logger().error('Failed to capture frame')

def main(args=None):
    rclpy.init(args=args)
    
    # Create and start the video publisher node
    publisher = VideoPublisher()
    print("Video Publisher Node started")
    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional since Python will call it automatically on garbage collection)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()