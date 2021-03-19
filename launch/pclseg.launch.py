import pathlib

from ament_index_python.packages import get_package_share_directory
from bps_launch_utils import bps_sensor_configs
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    launchdesc = LaunchDescription([])

    stereocam_cfgs = bps_sensor_configs(sensor_type='stereocam')

    zed2_lab_cfg = stereocam_cfgs['zed2_lab']

    launchdesc.add_action(Node(
        package='bps_dnn',
        executable='zed_cuda_node',
        parameters=[
            {
                'device': zed2_lab_cfg['device'],
                'name': zed2_lab_cfg['name'],
                'height': zed2_lab_cfg['height'],
                'width': zed2_lab_cfg['width']
            },
            zed2_lab_cfg['intrinsic'],
            {
                'model': str(
                    pathlib.Path(get_package_share_directory('bps_dnn'))
                    / 'networks'
                    / 'deeplabv3_resnet50_coco.onnx'
                )
            }]
    ))

    launchdesc.add_action(Node(
        package='livox',
        executable='livox_lidar_node',
        parameters=[{
            'ppm': 5800
        }]
    ))

    launchdesc.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.', '0.06', '0.05', '0.5', '-0.5', '0.5', '-0.5', 'livox', 'zed2_lab']
    ))

    launchdesc.add_action(Node(
        package='pcl_seg',
        executable='pcl_seg_node',
        parameters=[{
            'classes': [15]
        }],
        remappings=[
            ('calibration', 'left_calibration')
        ]
    ))

    launchdesc.add_action(Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', str(pathlib.Path(get_package_share_directory('bps_pcl_utils')
                                          ) / 'launch' / 'pclseg_view.rviz')]
    ))

    return launchdesc
