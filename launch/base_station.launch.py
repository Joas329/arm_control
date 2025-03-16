import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.descriptions import ParameterValue
import xacro


'''
                                                                  ┌───────────────┐     ┌────────────┐
                                   ┌───────────────────────┐  ┌───┼ /joint_states ◄─────┤UDP_Receiver│
                                   │         RVIZ2         │  │   └───────────────┘     └────────────┘
                                   │   ┌───────────────◄───┼──┘  ┌─────────────┐
                                   │   │  Arm Model 1  │   │  ┌──┤    Robot    │       ┌──────┐
                                   │   │(current State)◄───┼──┤  │ Description ◄───────┼ URDF │
                                   │   └───────────────┘   │  │  └─────────────┘       └──────┘
                                   │  ┌─────────────────┐  │  │
  ┌─────────────────────────┐      │  │   Arm Model 2   ◄──┼──┘ ┌────────────────┐    ┌─────────────────┐
  │joint_state_publisher_gui┼──────┼──►(desire position)┼──┼───►│new_joint_states┼───►│http_tranasmitter│
  └─────────────────────────┘      │  └─────────────────┘  │    └────────────────┘    └─────────────────┘
                                   └───────────────────────┘
'''
package_name='arm_control'

def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path,'description','base_station.urdf.xacro')

    # ****************** Robot 1 ****************** #

    static_transform_node1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'robot1_base_link'],
        output='screen'
    )

    robot1_description_command = Command([
        'xacro ', xacro_file,
        ' namespace:=robot1'
    ])

    robot1_description = {'robot_description': ParameterValue(robot1_description_command, value_type=str)}

    robot1_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="robot1",
        parameters=[robot1_description],
        remappings=[
            ('joint_states', '/decoded/joint_states')
        ],
        output="both",
    )

    joint_state_remapper_node = Node(
        package=package_name,
        executable="joint_states_remapping.py",
        output="both"
    )

    # ****************** Robot 2 ****************** #

    static_transform_node2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'robot2_base_link'],
        output='screen'
    )

    robot2_description_command = Command([
        'xacro ', xacro_file,
        ' namespace:=robot2'
    ])

    robot2_description = {'robot_description': ParameterValue(robot2_description_command, value_type=str)}

    robot2_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="robot2",
        parameters=[robot2_description],
        output="both",
    )

    # ****************** Joint State GUI ****************** #

    robot2_joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        namespace="robot2",
        parameters=[robot2_description],
        output="screen"
    )

    # ****************** Decoder Node ****************** #

    joint_decoder_spawner = Node(
        package=package_name,
        executable="base_station_decoder.py",
        output="both",
    )

    # ****************** Encoder Node ****************** #

    joint_encoder_spawner = Node(
        package=package_name,
        executable="base_station_encoder.py",
        output="both",
    )

    # ****************** UDP Transmitter Node ****************** #

    udp_transmitter = Node(
        package=package_name,
        executable="base_station_transmitter.py",
        output="both",
    )

    # ****************** UDP Receiver Node ****************** #

    udp_receiver = Node(
        package=package_name,
        executable="base_station_receiver.py",
        output="both",
    )

    # ****************** RVIZ 2 ****************** #

    rviz_config_file = os.path.join(pkg_path, 'config', '2arms.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        robot1_state_pub_node,
        #joint_state_remapper_node,
        static_transform_node1,
        joint_decoder_spawner,
        robot2_state_pub_node,
        robot2_joint_state_publisher_gui_node,
        static_transform_node2,
        joint_encoder_spawner,
        udp_transmitter,
        udp_receiver,
        rviz_node
    ])
