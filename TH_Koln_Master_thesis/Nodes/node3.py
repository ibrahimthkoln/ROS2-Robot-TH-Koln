# node3.py
import serial  # Reserved for future use when serial device is connected
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

class ServoControl(Node):
    """
    A ROS2 node that subscribes to an error value published by another node and simulates
    sending this value to a servo controller over a serial connection.
    """
    def __init__(self):
        super().__init__('rpi_to_arduino')
        # Subscription to receive error values from another node
        self.subscriber = self.create_subscription(Int16, '/cmd_servo_arduino', self.error_values_callback, 10)

        # Timer to periodically simulate sending error values to a serial device
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.error = 0  # Placeholder for the received error value

        #Serial device initialization commented out for future use
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

    def error_values_callback(self, value):
        """
        Callback function to update the current error value based on received data.
        """

        self.error = value.data


    def timer_callback(self):
        """
        Timer callback to simulate sending the current error value to a serial device.
        """
        # Simulating the creation of a message string to send over serial
        angle_value_list = [str(self.error)]
        send_string = ','.join(angle_value_list) + "\n"

        # Commented out serial communication for future use
        self.ser.write(send_string.encode('utf-8'))
        #receive_string = self.ser.readline().decode('utf-8', 'replace').rstrip()

        # For now, print the would-be sent string to the terminal
        print(f"Sending to serial: {send_string}")
        # Simulated response printout
        print("Simulated receive string: OK")

def main(args=None):
    rclpy.init(args=args)
    
    # Create and start the servo control node
    servo_control_node = ServoControl()
    rclpy.spin(servo_control_node)

    # Destroy the node explicitly
    servo_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
