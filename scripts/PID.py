#!/usr/bin/env python3

import numpy as np
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class PID(Node):

    def __init__(self):
        super().__init__("PID")

        # ***** Node Variables Here ******* #
        self.initVariablesFlag = False

        # ******** Timers ******** #
        self.velocityPIDTimer = self.create_timer(0.02,self.velocityPID) #TODO YOU CAN CHANGE THE TIMER TIME HERE
        self.clipub = self.create_timer(3,self.clipublisher)


        # ******** Subscribers ******** #
        self.desired_joint_position_subscriber = self.create_subscription(
            JointState,
            'decoded/desired_joint_states',
            self.desired_position_callback,
            10
        )

        self.current_joint_state_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.current_position_callback,
            10
        )

        # *************** Publishers *************** #
        self.arm_commands_publisher = self.create_publisher(
            Float64MultiArray,
            'arm_group_controller/commands',
            10
        )

    # ******************* Timed Callbacks ******************** #

    def velocityPID(self):

        if self.initVariablesFlag == True:

            velocities = self.pidVelocityCalc(self.desired_joint_positions,self.joints_number)
            self.arm_commands_publisher.publish(velocities)

    def pidVelocityCalc(self, jointDesiredPositions, joints_number):
        velocityCommand = Float64MultiArray()

        k_pose_p = self.get_parameter('k_pose_p').get_parameter_value().double_array_value
        k_pose_i = self.get_parameter('k_pose_i').get_parameter_value().double_array_value
        k_pose_d = self.get_parameter('k_pose_d').get_parameter_value().double_array_value

        joint = 0

        current_pose_error = [0.1] * joints_number
        velocity_sum = [0.1] * joints_number
        previous_pose_error = [0.1] * joints_number
        p_pose_factor = [0.1] * joints_number
        i_pose_factor = [0.1] * joints_number
        d_pose_factor = [0.1] * joints_number

        for desiredPose in jointDesiredPositions:
            current_pose_error[joint] = desiredPose - self.ordered_current_pose[joint]
            #P function
            p_pose_factor[joint] = k_pose_p[joint] * (current_pose_error[joint])
            #I function
            i_pose_factor[joint] = k_pose_i[joint] * ((velocity_sum[joint] + current_pose_error[joint]))
            velocity_sum[joint] += current_pose_error[joint]
            #D function
            d_pose_factor[joint] = k_pose_d[joint] * (current_pose_error[joint] - previous_pose_error[joint])
            previous_pose_error[joint] = current_pose_error[joint]
            #publish this value to the respective joint
            self.velocity[joint] = p_pose_factor[joint] + i_pose_factor[joint] + d_pose_factor[joint]
            velocityCommand.data.append(self.velocity[joint])
            joint+= 1
        return velocityCommand

    # ********** Subcriber Callbacks *********** #

    def desired_position_callback(self, desire_joint_states):

        if self.initVariablesFlag == True:
            self.desired_joint_positions = desire_joint_states.position
        return

    def current_position_callback(self, current_joint_states):
        if self.initVariablesFlag == False:
            self.initVariables(current_joint_states)
        else:
            # Calculte velocity based on position changes
            self.readingTime = time.time()
            self.poseToVel()

            # Order joints by name
            self.currentPosition = current_joint_states.position
            for i in range(0,self.joints_number -1):
                self.ordered_current_pose[i] = self.currentPosition[self.joint_order[i]]
        return

    # *************Velocity derivative*************** #
    def poseToVel(self):
        t = time.time() - self.readingTime

        for i in range(0,5):
            self.curr_vel[i] = (self.curr_vel[i] - self.prev_curr_vel[i]) / t

    def initVariables(self, joints):
        color_start = "\033[32m"
        color_reset = "\033[0m"
        self.get_logger().info(f"{color_start}STARTING PID NODE{color_reset}")
        self.joints_number = len(joints.name)
        self.joint_names = joints.name
        self.joint_order = [0] * self.joints_number

        self.ordered_current_pose = self.curr_vel = self.prev_curr_vel = self.currentPosition = self.desired_joint_positions  = [0.0] * self.joints_number
        self.command = self.velocity  = [0.0] * self.joints_number

        #Parameter Definition
        self.declare_parameter("k_pose_p", [0.0] * self.joints_number)
        self.declare_parameter("k_pose_i", [0.0] * self.joints_number)
        self.declare_parameter("k_pose_d", [0.0] * self.joints_number)
        self.declare_parameter("k_vel_p", [0.0] * self.joints_number)
        self.declare_parameter("k_vel_i", [0.0] * self.joints_number)
        self.declare_parameter("k_vel_d", [0.0] * self.joints_number)

        self.dynamicJointCheck(joints.name)

        self.initVariablesFlag = True

        return

    def dynamicJointCheck(self, jointStates):
        counter = 0
        for joint in jointStates:
            self.joint_order[int(joint[len(joint)-1])-1] = counter
            counter += 1

        return

    def clipublisher(self):

        if self.initVariablesFlag == True:

            k_pose_p = self.get_parameter('k_pose_p').get_parameter_value().double_array_value
            k_pose_i = self.get_parameter('k_pose_i').get_parameter_value().double_array_value
            k_pose_d = self.get_parameter('k_pose_d').get_parameter_value().double_array_value
            k_vel_p = self.get_parameter('k_vel_p').get_parameter_value().double_array_value
            k_vel_i = self.get_parameter('k_vel_i').get_parameter_value().double_array_value
            k_vel_d = self.get_parameter('k_vel_d').get_parameter_value().double_array_value


            self.get_logger().info("number of detected joints: " + str(self.joints_number))
            self.get_logger().info("detected joints: " + str(self.joint_names)) # need to cahnge this to names instead of values
            self.get_logger().info("***********Velocity**********")
            self.get_logger().info("P values: " + str(k_pose_p.tolist()))
            self.get_logger().info("I values: " + str(k_pose_i.tolist()))
            self.get_logger().info("D values: " + str(k_pose_d.tolist()))
            self.get_logger().info("")
            self.get_logger().info("***********Effort**********")
            self.get_logger().info("P values: " + str(k_vel_p.tolist()))
            self.get_logger().info("I values: " + str(k_vel_i.tolist()))
            self.get_logger().info("D values: " + str(k_vel_d.tolist()))
            self.get_logger().info("***********Commands**********")
            self.get_logger().info("commands" + str((self.command)))
            color_start = "\033[35m"
            color_reset = "\033[0m"
            self.get_logger().info("**********Current Position*************")
            self.get_logger().info(', '.join(f"{color_start}{x:.2f}" for x in self.ordered_current_pose))
            self.get_logger().info("**********Desired Position*************")
            self.get_logger().info(', '.join(f"{x:.2f}" for x in self.desired_joint_positions))
            self.get_logger().info(f"\n \n{color_reset}")


def main(args=None):
    rclpy.init(args=args)
    node = PID()
    color_start = "\033[34m"
    color_reset = "\033[0m"
    node.get_logger().info(f"{color_start}Initializing PID and waiting on desired positions{color_reset}")
    rclpy.spin(node)
    node.timer.cancel()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
