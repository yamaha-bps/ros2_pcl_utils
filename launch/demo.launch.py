# Copyright Yamaha 2021
# MIT License
# https://github.com/yamaha-bps/ros2_pcl_utils/blob/master/LICENSE

from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    desc = LaunchDescription([
    ])

    desc.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments=[('world', 'worlds/pcl_demo.world')]
        )
    )

    desc.add_action(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.1', '-1.57079632', '0', '-1.57079632', 'lidar', 'camera'],
            parameters=[{
                'use_sim_time': True
            }]
        )
    )

    desc.add_action(
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=[
                '-d', join(get_package_share_directory('pcl_utils'), 'launch', 'demo.rviz')
            ],
            parameters=[{
                ''
                'use_sim_time': True
            }]
        )
    )

    desc.add_action(
        Node(
            package='pcl_utils',
            executable='feature_node',
            parameters=[{
                'ang_disc_thresh': 3.14,
                'ang_inc_thresh': 60 * 3.14 / 180,
                'edge_thresh': 0.005,
                'max_angle': 25 * 3.14 / 180,
                'max_depth': 15.,
                'max_intensity': 1.,
                'min_depth': 0.0,
                'min_intensity': 0.0,
                'plane_thresh': 0.0005,
                'rel_disc_thresh': 0.05,
                'window': 5,
                'use_sim_time': True
            }],
            remappings=[
                ('pointcloud', 'sensors/lidar/out')]

        )
    )

    desc.add_action(
        Node(
            package='pcl_utils',
            executable='image_overlay_node',
            remappings=[
                ('pointcloud', 'sensors/lidar/out'),
                ('image', 'sensors/camera/image_raw'),
                ('camera_info', 'sensors/camera/camera_info')
            ],
            parameters=[{
                'cmap_min': 4.,
                'cmap_max': 15.,
                'draw_radius': 2,
                'use_sim_time': True
            }]
        )
    )

    return desc
