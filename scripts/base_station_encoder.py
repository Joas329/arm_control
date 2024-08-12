#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import json

class JointStateStringifier(Node):
    def __init__(self):
        super().__init__('joint_state_stringifier')
        self.subscription = self.create_subscription(
            JointState,
            '/robot2/joint_states',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(
            String,
            '/robot2/raw_strings_joint_states',
            10)

    def listener_callback(self, msg):
        joint_data = {
            "header": {
                "stamp": {
                    "sec": msg.header.stamp.sec,
                    "nanosec": msg.header.stamp.nanosec
                },
                "frame_id": msg.header.frame_id
            },
            "names": list(msg.name),
            "positions": list(msg.position),
            "velocities": list(msg.velocity),
            "efforts": list(msg.effort)
        }
        json_data = json.dumps(joint_data)
        str_msg = String()
        str_msg.data = json_data
        self.publisher.publish(str_msg)
        
def main(args=None):
    rclpy.init(args=args)
    joint_state_stringifier = JointStateStringifier()
    
    color_start = "\033[35m"
    color_reset = "\033[0m"
    joint_state_stringifier.get_logger().info(f"{color_start}Encoding /robot2/joint_states to '/robot2/raw_strings_joint_states{color_reset}")

    rclpy.spin(joint_state_stringifier)
    joint_state_stringifier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
