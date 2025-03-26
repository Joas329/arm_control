import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
from ament_index_python import get_package_prefix


package_name='arm_control'

pkg_share_path = os.pathsep + os.path.join(get_package_prefix(package_name), 'share')
if 'GAZEBO_MODEL_PATH' in os.environ:
    os.environ['GAZEBO_MODEL_PATH'] += pkg_share_path
else:
    os.environ['GAZEBO_MODEL_PATH'] =  pkg_share_path


def generate_launch_description():

    print("Launching arm simulation part: ")

    # ****************** Robot Description ****************** #
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path,'description','arm_simulation.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # ****************** Robot State Publisher (joint_states) ****************** #
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )

    # ****************** Gazebo ****************** #
    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.world'
        )

    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )

    gzserver_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
                )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                            '-entity', 'robot_arm',
                            '-name', 'robot_arm',
                            '-z', '0.1'],
        output='screen'
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

    return LaunchDescription([
        robot_state_pub_node,
        world_arg,
        control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        joint_encoder_spawner,
        joint_decoder_spawner,
        udp_receiver,
        udp_transmitter,
        PID_scripts_spawner,
        gzserver_cmd,
        spawn_entity
    ])
