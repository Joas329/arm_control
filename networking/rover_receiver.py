#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket

class UdpReceiverNode(Node):
    def __init__(self):
        super().__init__('udp_receiver_node')
        self.declare_parameter('udp_port', 5001)
        port = self.get_parameter('udp_port').get_parameter_value().integer_value

        # Set up the UDP socket to non-blocking mode
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('0.0.0.0', port))
        self.socket.setblocking(False)

        # publisher
        self.publisher = self.create_publisher(String, '/udp_receiver', 10)
        
        # timer callback
        self.timer = self.create_timer(0.1, self.listen)

    def listen(self):
        try:
            data, addr = self.socket.recvfrom(1024)  # Buffer size of 1024 bytes
            if data:
                message = data.decode('utf-8')
                msg = String()
                msg.data = message
                self.publisher.publish(msg)

        except BlockingIOError:
            pass  # No data received, ignore and continue

def main(args=None):
    rclpy.init(args=args)
    node = UdpReceiverNode()
    color_start = "\033[35m"
    color_reset = "\033[0m"
    node.get_logger().info(f"{color_start}Listening on port: 5001, for UDP packets{color_reset}")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
