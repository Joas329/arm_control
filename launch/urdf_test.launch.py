import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

package_name='arm_control'

def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory(package_name))
    urdf_file = os.path.join(pkg_path,'description','robotic_arm_4.urdf')

    state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': open(urdf_file).read()}],
        output="both",
    )

    # ****************** Joint State GUI ****************** #

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[{'robot_description': open(urdf_file).read()}],
        output="screen"
    )

    # ****************** RVIZ 2 ****************** #

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return LaunchDescription([
        state_pub_node,
        joint_state_publisher_gui_node,
	    rviz_node
    ])
