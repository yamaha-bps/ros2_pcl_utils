import pathlib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    launchdesc = LaunchDescription([])

    launchdesc.add_action(Node(
        package='livox',
        executable='livox_lidar_node',
        parameters=[{
            'ppm': 5000,
            'skip_noisy': False,
            'num_lasers': 6
        }]
    ))

    launchdesc.add_action(Node(
        package='bps_pcl_utils',
        executable='pcl_feature_node',
        parameters=[{
            'ang_disc_thresh': 3.14,
            'ang_inc_thresh': 20 * 3.14 / 180,
            'edge_thresh': 0.04,
            'max_angle': 25 * 3.14 / 180,
            'max_depth': 5.,
            'max_intensity': 1.,
            'min_depth': 0.0,
            'min_intensity': 0.0,
            'plane_thresh': 0.0005,
            'rel_disc_thresh': 0.025,
            'window': 5,
        }]
    ))

    launchdesc.add_action(Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', str(pathlib.Path(get_package_share_directory('bps_pcl_utils')
                                          ) / 'launch' / 'pclfeature_view.rviz')]
    ))

    return launchdesc
