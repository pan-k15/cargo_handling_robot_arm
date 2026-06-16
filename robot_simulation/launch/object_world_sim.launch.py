from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ── Robot description: robot_2 with depth camera ──────────────────
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([
                FindPackageShare('robot_description'), 'urdf', 'robot_2.urdf.xacro'
            ]),
        ]),
        value_type=str,
    )
    robot_description = {'robot_description': robot_description_content}

    robot_controllers = PathJoinSubstitution([
        FindPackageShare('robot_simulation'), 'config', 'ros2_controllers.yaml'
    ])

    object_world = PathJoinSubstitution([
        FindPackageShare('robot_simulation'), 'worlds', 'object.sdf'
    ])

    # ── Core nodes ────────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name',  'arm6dof',
            '-allow_renaming', 'true',
        ],
    )

    # ── Controller spawners ───────────────────────────────────────────
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--param-file', robot_controllers],
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--param-file', robot_controllers],
    )

    # ── Bridges ───────────────────────────────────────────────────────
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        arguments=[
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/camera/image',       '/camera/color/image_raw'),
            ('/camera/depth_image', '/camera/depth/image_raw'),
            ('/camera/points',      '/camera/depth/points'),
            ('/camera/camera_info', '/camera/color/camera_info'),
        ],
        output='screen',
    )

    # ── MoveIt2 move_group (from moveit_config_2) ─────────────────────
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('moveit_config_2'), 'launch', 'move_group.launch.py'
            ])
        ]),
    )

    # ── Pick / place service servers ──────────────────────────────────
    pick_server = Node(
        package='robot_services',
        executable='pick_service_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    place_server = Node(
        package='robot_services',
        executable='place_service_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── Vision: YOLO classifier ───────────────────────────────────────
    yolo_classifier = Node(
        package='robot_vision',
        executable='yolo_classifier.py',
        name='yolo_classifier',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── Vision: Sort orchestrator ─────────────────────────────────────
    sort_orchestrator = Node(
        package='robot_vision',
        executable='sort_orchestrator.py',
        name='sort_orchestrator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── RViz ──────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('moveit_config_2'), 'config', 'moveit.rviz'
        ])],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo simulated clock',
        ),

        # Gazebo with object world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                ])
            ]),
            launch_arguments=[('gz_args', ['-r -v 1 ', object_world])],
        ),

        clock_bridge,
        camera_bridge,
        robot_state_publisher,
        rviz,
        spawn_robot,

        # Controllers start after robot is spawned
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner, gripper_controller_spawner],
            )
        ),

        # MoveIt2, services and vision start after controllers are ready
        RegisterEventHandler(
            OnProcessExit(
                target_action=arm_controller_spawner,
                on_exit=[
                    move_group,
                    pick_server,
                    place_server,
                    yolo_classifier,
                    sort_orchestrator,
                ],
            )
        ),
    ])
