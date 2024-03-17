import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

import math

timer_T = 0.1

class SignalGenerator(Node):

    def __init__(self):
        super().__init__('signal_generator')
        self.signal_publisher_ = self.create_publisher(Float32, 'signal', 10)
        self.time_publisher_ = self.create_publisher(Float32, 'time', 10)
        timer_period = timer_T  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0.0

    def timer_callback(self):
        time_msg = Float32()
        time_msg.data = self.i
        self.time_publisher_.publish(time_msg)

        sine_msg = Float32()
        sine_msg.data = math.sin(time_msg.data)
        self.signal_publisher_.publish(sine_msg)

        self.get_logger().info('Time: "%f", sin(t)= "%f"' % (time_msg.data, sine_msg.data))
        # self.get_logger().info('Time: "%f"' % time_msg.data)
        self.i += timer_T


def main(args=None):
    rclpy.init(args=args)

    signal_generator = SignalGenerator()

    rclpy.spin(signal_generator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    signal_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()