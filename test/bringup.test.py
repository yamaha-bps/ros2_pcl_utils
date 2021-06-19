import unittest

from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import pytest


@pytest.mark.launch_test
def generate_test_description():

    feature_node = Node(package='pcl_utils', executable='feature_node')
    seg_node = Node(package='pcl_utils', executable='seg_node')
    overlay_node = Node(package='pcl_utils', executable='image_overlay_node')

    desc = LaunchDescription([
        feature_node,
        seg_node,
        overlay_node,
        launch_testing.actions.ReadyToTest()
    ])

    context = {
        'feature_node': feature_node,
        'seg_node': seg_node,
        'overlay_node': overlay_node
    }

    return desc, context


class TestStarted(unittest.TestCase):

    def test_feature(self, proc_output, feature_node):
        proc_output.assertWaitFor(
            'Started node',
            process=feature_node,
            timeout=30
        )

    def test_seg(self, proc_output, seg_node):
        proc_output.assertWaitFor(
            'Started node',
            process=seg_node,
            timeout=30
        )

    def test_overlay(self, proc_output, overlay_node):
        proc_output.assertWaitFor(
            'Started node',
            process=overlay_node,
            timeout=30
        )


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_info, feature_node):

        launch_testing.asserts.assertExitCodes(proc_info, process=feature_node)
