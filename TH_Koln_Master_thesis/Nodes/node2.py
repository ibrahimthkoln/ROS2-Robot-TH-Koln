# node2.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import serial

class CameraSubscriber(Node):
    """
    A ROS2 node that subscribes to video frames, processes them to detect edges, 
    and calculates a specific error based on the position of detected edges.
    It publishes both the processed image and the calculated error.
    """
    def __init__(self):
        super().__init__('video_subscriber')
        # Subscription to video frames
        self.camera_subscriber = self.create_subscription(Image, '/rpi_video', self.camera_callback, 10)
        self.bridge = CvBridge()

        self.error = 0
        
        # Publisher for the calculated error
        self.error_publisher = self.create_publisher(Int16, '/cmd_servo_arduino', 10)
        # Publisher for the processed image
        self.image_publisher = self.create_publisher(Image, '/processed_video', 10)
        
        # Timer for periodically publishing the error
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def camera_callback(self, data):
        """
        Callback function for processing incoming video frames.
        """
        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(data, 'mono8')
        # Edge detection
        edged = cv2.Canny(frame, 80, 100)

        white_index = []
        mid_point_line = 0
        # Detect white pixels in the middle row to find edges
        for index, value in enumerate(edged[:][139]):
            if value == 255:
                white_index.append(index)
        #--------------------------------------------------------------------------
        if(len(white_index)==4):
            white_index[0]=white_index[0]
            white_index[1]=white_index[2]
            
        ## When Values are 3 and if they are 2 we donot need to process them
        ## If the values of pixels is greater then 5 then they are adjecent to eachother
        if(len(white_index)==3):
            for i in range(len(white_index)):
                if(white_index[1]-white_index[0] > 5): ## meaning idx(0) and idx(1) are ok [193, 506, 507 ]
                    white_index[0]=white_index[0]
                    white_index[1]=white_index[1]
                else:#[193, 194, 507 ]
                    white_index[0]=white_index[0]
                    white_index[1]=white_index[2]

        ##=----------------------------------------------------------------------------------------------------


        # If two edges are detected, calculate their midpoint
        if len(white_index) == 2:
            cv2.circle(img=edged, center=(white_index[0], 139), radius=2, color=(255, 0, 0), thickness=1)
            cv2.circle(img=edged, center=(white_index[1], 139), radius=2, color=(255, 0, 0), thickness=1)
            mid_point_line = int((white_index[0] + white_index[1]) / 2)
            cv2.circle(img=edged, center=(mid_point_line, 139), radius=3, color=(255, 0, 0), thickness=2)

        # Calculate error based on the robot's midpoint and the line's midpoint
        mid_point_robot = [205, 139]  # Assumed robot's midpoint
        cv2.circle(img=edged, center=(mid_point_robot[0], mid_point_robot[1]), radius=5, color=(255, 0, 0), thickness=2)
        self.error = mid_point_robot[0] - mid_point_line
       # if self.error == 205:
        #   self.error=0
        
        # Convert the processed frame back to a ROS Image message and publish it
        processed_frame_msg = self.bridge.cv2_to_imgmsg(edged, "mono8")
        self.image_publisher.publish(processed_frame_msg)

    def timer_callback(self):
        """
        Timer callback for periodically publishing the calculated error.
        """
        msg = Int16()
        msg.data = self.error
        self.error_publisher.publish(msg)
        self.get_logger().info(f"CMD -> {msg.data}")
        

def main(args=None):
    rclpy.init(args=args)

    # Create and start the video subscriber node
    camera_subscriber = CameraSubscriber()
    print("Video Subscriber Node started")
    rclpy.spin(camera_subscriber)
    
    # Destroy the node explicitly
    camera_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()