import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robot_description')

    urdf_file = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')
    rviz_file = os.path.join(pkg, 'rviz', 'display.rviz')

    robot_description = xacro.process_file(urdf_file).toxml()

    return LaunchDescription([
        # publishes /robot_description and TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False,
            }],
        ),

        # interactive sliders for every non-fixed joint
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),

        # RViz2 with pre-baked config
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
        ),
    ])
