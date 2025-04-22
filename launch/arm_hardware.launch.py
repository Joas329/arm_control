from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution


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
    serial_port = LaunchConfiguration("serial_port")
    calibrate = LaunchConfiguration("calibrate")
    include_gripper = LaunchConfiguration("include_gripper")
    arduino_serial_port = LaunchConfiguration("arduino_serial_port")
    arm_model_config = LaunchConfiguration("arm_model")
    tf_prefix = LaunchConfiguration("tf_prefix")

    print("Launching arm part: ")

    # ****************** Robot Description ****************** #
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [FindPackageShare("arm_control"), "description", "hardware_launch.urdf.xacro"]),
        " ",
        "arm_model:=",
        arm_model_config,
        " ",
        "serial_port:=",
        serial_port,
        " ",
        "calibrate:=",
        calibrate,
        " ",
        "tf_prefix:=",
        tf_prefix,
        " ",
        "include_gripper:=",
        include_gripper,
        " ",
        "arduino_serial_port:=",
        arduino_serial_port,
    ])

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # ****************** Robot State Publisher (joint_states) ****************** #

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )

    # ****************** ROS2 Controllers (ros2 controllers) ****************** #
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            update_rate_config_file,
            ParameterFile(joint_controllers_cfg, allow_substs=True),
            {
                "tf_prefix": tf_prefix
            },
        ],
        remappings=[('~/robot_description', 'robot_description')],
        output="screen",
    )

    joint_controllers_cfg = PathJoinSubstitution(
        [FindPackageShare("arm_control"), "config", "arm_controllers.yaml"])

    update_rate_config_file = PathJoinSubstitution([
        FindPackageShare("annin_ar4_driver"),
        "config",
        "controller_update_rate.yaml",
    ])

    arm_controller_spawner =  Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "100",
        ],
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
        executable="arm_states_encoder.py",
        output="both",
    )

    # ****************** Decoder Node ****************** #

    joint_decoder_spawner = Node(
        package=package_name,
        executable="arm_states_decoder.py",
        output="both",
    )

    # ****************** UDP Transmitter Node ****************** #

    udp_transmitter = Node(
        package=package_name,
        executable="arm_transmitter.py",
        output="both",
    )

    # ****************** UDP Receiver Node ****************** #

    udp_receiver = Node(
        package=package_name,
        executable="arm_receiver.py",
        output="both",
    )

    # ****************** PID Node (VelPID node) ****************** #

    PID_scripts_spawner = Node(
        package=package_name,
        executable="PID.py",
        output="both",
    )

    # ****************** Launch Description ****************** #
    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyACM0",
            description="Serial port to connect to the robot",
        ))
    ld.add_action(
        DeclareLaunchArgument(
            "calibrate",
            default_value="False",
            description="Calibrate the robot on startup",
            choices=["True", "False"],
        ))
    ld.add_action(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="Prefix for AR4 tf_tree",
        ))
    ld.add_action(
        DeclareLaunchArgument(
            "include_gripper",
            default_value="True",
            description="Run the servo gripper",
            choices=["True", "False"],
        ))
    ld.add_action(
        DeclareLaunchArgument(
            "arduino_serial_port",
            default_value="/dev/ttyUSB0",
            description="Serial port of the Arduino nano for the servo gripper",
        ))
    ld.add_action(
        DeclareLaunchArgument("arm_model",
                              default_value="mk3",
                              choices=["mk1", "mk2", "mk3"],
                              description="Model of AR4"))
    ld.add_action(controller_manager_node)
    ld.add_action(arm_controller_spawner)
    # ld.add_action(gripper_controller_spawner)
    ld.add_action(robot_state_pub_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(joint_encoder_spawner)
    ld.add_action(joint_decoder_spawner)
    ld.add_action(joint_decoder_spawner)
    ld.add_action(udp_receiver)
    ld.add_action(udp_transmitter)
    ld.add_action(PID_scripts_spawner)

    return ld
