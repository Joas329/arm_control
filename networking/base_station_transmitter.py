#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket

class UdpStringSender(Node):
    def __init__(self):
        super().__init__('udp_string_sender')
        self.declare_parameter('udp_hostname', 'localhost')
        self.declare_parameter('udp_port', 5001)
        
        hostname = self.get_parameter('udp_hostname').get_parameter_value().string_value
        port = self.get_parameter('udp_port').get_parameter_value().integer_value
        
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.address = (hostname, port)

        self.subscription = self.create_subscription(
            String,
            '/robot2/raw_strings_joint_states',
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self, msg):
        data = msg.data.encode('utf-8')
        self.socket.sendto(data, self.address)

def main(args=None):
    rclpy.init(args=args)
    node = UdpStringSender()
    color_start = "\033[35m"
    color_reset = "\033[0m"
    node.get_logger().info(f"{color_start}Transmitting on port: 5001, for UDP packets{color_reset}")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
