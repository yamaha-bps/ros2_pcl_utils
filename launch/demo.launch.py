from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


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

    return desc
