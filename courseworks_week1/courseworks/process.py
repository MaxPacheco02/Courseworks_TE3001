import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

import math

timer_T = 0.1
alpha = 0.5

class Process(Node):
    processed_signal_msg = Float32()

    def __init__(self):
        super().__init__('process')
        self.proc_signal_publisher_ = self.create_publisher(Float32, 'proc_signal', 10)
        timer_period = timer_T  # seconds

        self.signal_subscriber_ = self.create_subscription(
            Float32,
            'signal',
            self.signal_callback,
            10)
        self.time_subscriber_ = self.create_subscription(
            Float32,
            'time',
            self.time_callback,
            10)

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0.0

    def timer_callback(self):
        self.proc_signal_publisher_.publish(self.processed_signal_msg)

    def signal_callback(self, msg):
        # self.processed_signal_msg.data = msg.data / 2 + alpha
        self.get_logger().info('Original Signal= "%f", Processed Signal= "%f"' % (msg.data, self.processed_signal_msg.data))

    def time_callback(self, msg):
        self.processed_signal_msg.data = math.sin(msg.data - 1.1) / 2 + alpha

def main(args=None):
    rclpy.init(args=args)

    process = Process()

    rclpy.spin(process)

    process.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()