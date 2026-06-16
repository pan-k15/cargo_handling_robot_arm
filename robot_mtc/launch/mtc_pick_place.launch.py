"""
mtc_pick_place.launch.py

Starts the MTC pick-and-place node alongside move_group.
Requires the simulation (object_world_sim.launch.py) to be running first.

Usage:
  ros2 launch robot_mtc mtc_pick_place.launch.py

Then trigger a pick-and-place:
  ros2 service call /mtc_pick_place robot_interfaces/srv/Pick \
    "{target_pose: {position: {x: 0.5, y: 0.0, z: 0.15}}, approach_height: 0.1}"

Override place pose via parameters:
  ros2 launch robot_mtc mtc_pick_place.launch.py place_x:=0.35 place_y:=0.50 place_z:=0.07
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    place_x = LaunchConfiguration('place_x', default='0.35')
    place_y = LaunchConfiguration('place_y', default='0.50')
    place_z = LaunchConfiguration('place_z', default='0.07')

    # ── move_group (MoveIt 2) ──────────────────────────────────────────────
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('moveit_config_2'),
                'launch', 'move_group.launch.py'
            ])
        ])
    )

    # ── MTC pick-and-place node ────────────────────────────────────────────
    mtc_node = Node(
        package='robot_mtc',
        executable='pick_place_mtc',
        name='mtc_pick_place',
        output='screen',
        parameters=[{
            'place_x': place_x,
            'place_y': place_y,
            'place_z': place_z,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('place_x', default_value='0.35',
                              description='Place target X (m)'),
        DeclareLaunchArgument('place_y', default_value='0.50',
                              description='Place target Y (m)'),
        DeclareLaunchArgument('place_z', default_value='0.07',
                              description='Place target Z — landing surface (m)'),
        move_group,
        mtc_node,
    ])
