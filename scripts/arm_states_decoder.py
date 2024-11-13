#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import json

class JointStateDecoder(Node):
    def __init__(self):
        super().__init__('joint_state_decoder')
        self.subscription = self.create_subscription(
            String,
            '/udp_receiver',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(
            JointState,
            '/decoded/desired_joint_states',
            10)

    def listener_callback(self, msg):
        joint_data = json.loads(msg.data)

        joint_state_msg = JointState()
        joint_state_msg.header.stamp.sec = joint_data["header"]["stamp"]["sec"]
        joint_state_msg.header.stamp.nanosec = joint_data["header"]["stamp"]["nanosec"]
        joint_state_msg.header.frame_id = joint_data["header"]["frame_id"]

        joint_state_msg.name = [name.replace('robot2_', '') for name in joint_data["names"]]

        joint_state_msg.position = joint_data["positions"]
        joint_state_msg.velocity = joint_data["velocities"]
        joint_state_msg.effort = joint_data["efforts"]

        self.publisher.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)
    decoder = JointStateDecoder()
    color_start = "\033[35m"
    color_reset = "\033[0m"
    decoder.get_logger().info(f"{color_start}Decoding /udp_receiver (arm) to /decoded/desired_joint_states{color_reset}")
    rclpy.spin(decoder)
    decoder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
