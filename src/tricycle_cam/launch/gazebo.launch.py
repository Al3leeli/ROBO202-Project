import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('tricycle_cam')

    tricycle_urdf_path = os.path.join(pkg_share, 'urdf', 'tricycle.urdf')
    obstacles_urdf_path = os.path.join(pkg_share, 'urdf', 'obstacle.urdf')

    # Load tricycle URDF into robot_description
    with open(tricycle_urdf_path, 'r') as f:
        tricycle_description = f.read()

    return LaunchDescription([

        # 1) Start Gazebo (empty world)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                )
            ),
        ),

        # 2) Robot State Publisher for tricycle
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_tricycle',
            output='screen',
            parameters=[{'robot_description': tricycle_description}]
        ),

        # 3) Spawn tricycle from robot_description
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_tricycle',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'tricycle_robot',
                '-x', '0', '-y', '0', '-z', '0.1'
            ],
        ),

        # 4) Spawn ALL obstacles from obstacle.urdf (one entity)
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_obstacles',
            output='screen',
            arguments=[
                '-file', obstacles_urdf_path,
                '-entity', 'obstacles',
                '-x', '2', '-y', '0', '-z', '0.1'
            ],
        ),

        # 5) Image subscriber node
        Node(
            package='tricycle_cam',
            executable='image_subscriber',
            name='image_subscriber',
            output='screen',
            parameters=[{'save_dir': os.path.expanduser('~/camera/image_raw'), 'save_cooldown': 5.0}],
        ),
    ])
