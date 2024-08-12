import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from moveit_configs_utils import MoveItConfigsBuilder
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import yaml


'''
This file launches all necessary scripts to manipulate the arm from base station.
The following architechture is:

        ┌─────────────────┐       ┌──────────────┐       ┌─────────────────────────┐
        │ UDP_Transmitter ◄───────┼ joint_states ◄───────┼Hardware_state_interfaces│
        └─────────────────┘       └──┬───────────┘       └─────────────────────────┘
                                     │
        ┌───────────────┐   ┌────────▼────┐      ┌───────────────┐     ┌────────┐
        │ HTTP_Receiver ┼───► VelPID node─┼──────►ros2 Contollers┼─────►Hardware│
        └───────────────┘   └─────────────┘      └───────────────┘     └────────┘


'''

package_name='arm_control'

def generate_launch_description():

    print("Launching arm rover part: ")

    # ****************** Robot Description ****************** #

    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path,'description','hardware_launch.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # ****************** Robot State Publisher (joint_states) ****************** #

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )

    # ****************** ROS2 Controllers (ros2 controllers) ****************** #

    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','arm_controllers.yaml')
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_params_file],
        output="both",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_group_controller", "--controller-manager", "/controller_manager"],
        output="both",
    )

    # ****************** Joint State Broadcaster ****************** #

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="both",
    )

    # ****************** Encoder Node ****************** #

    joint_encoder_spawner = Node(
        package=package_name,
        executable="rover_states_encoder.py",
        output="both",
    )

    # ****************** Decoder Node ****************** #

    joint_decoder_spawner = Node(
        package=package_name,
        executable="rover_states_decoder.py",
        output="both",
    )

    # ****************** UDP Transmitter Node ****************** #

    udp_transmitter = Node(
        package=package_name,
        executable="rover_transmitter.py",
        output="both",
    )

    # ****************** UDP Receiver Node ****************** #

    udp_receiver = Node(
        package=package_name,
        executable="rover_receiver.py",
        output="both",
    )

    # ****************** PID Node (VelPID node) ****************** #

    PID_scripts_spawner = Node(
        package=package_name,
        executable="PID.py",
        output="both",
    )

    return LaunchDescription([
        robot_state_pub_node,
        control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        joint_encoder_spawner,
        joint_decoder_spawner,
        udp_receiver,
        udp_transmitter,
        PID_scripts_spawner
    ])
