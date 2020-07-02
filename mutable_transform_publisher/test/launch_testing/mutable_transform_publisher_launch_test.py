import os
import unittest

from ament_index_python import get_package_share_directory, get_package_prefix
import shutil
import tempfile

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time

import launch
import launch.actions
import launch_ros.actions


import launch_testing
import launch_testing.actions

import tf2_ros
from geometry_msgs.msg import TransformStamped
from mutable_transform_publisher_msgs.srv import SetTransform

import pytest


@pytest.mark.launch_test
def generate_test_description():
    # copy .YAML with test tform to /tmp directory
    yaml_path = os.path.join(tempfile.gettempdir(), 'test_tform.yaml')
    shutil.copy(os.path.join(get_package_share_directory('mutable_transform_publisher'), 'test', 'config', 'one_tform.yaml'), yaml_path)

    mutable_tf_pub = launch_ros.actions.Node(
         node_name='mutable_tf_pub',
         package='mutable_transform_publisher',
         node_executable='mutable_transform_pub',
         additional_env={'PYTHONUNBUFFERED': '1'},
         parameters=[{'yaml_path': yaml_path}]
    )

    return launch.LaunchDescription([
        mutable_tf_pub,
        launch_testing.actions.ReadyToTest(),
    ])


class TestSetTransforms(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS node for tests
        self.node = rclpy.create_node('test_mutable_tf_pub')
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self.node)

    def tearDown(self):
        self.node.destroy_node()

    def test_tf_lookup(self, proc_output):
        # spin for a few cycles until the transform is available
        n = 0
        while True:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            try:
                self.buffer.lookup_transform('foo', 'bar', Time(seconds=0), timeout=Duration(seconds=0, nanoseconds=1e8))
                break
            except tf2_ros.LookupException:
                n += 1
                if n > 10:
                    raise

        tform = self.buffer.lookup_transform('foo', 'bar', Time(seconds=0), timeout=Duration(seconds=5))

        # make sure that the transform received matches what we expect
        assert tform.header.frame_id == 'foo'
        assert tform.child_frame_id == 'bar'
        assert tform.transform.translation.x == 0.0
        assert tform.transform.translation.y == 1.0
        assert tform.transform.translation.z == 2.0
        assert tform.transform.rotation.w == 1.0
        assert tform.transform.rotation.x == 0.0
        assert tform.transform.rotation.y == 0.0
        assert tform.transform.rotation.z == 0.0

        tform_client = self.node.create_client(SetTransform, "/set_transform")

        req = SetTransform.Request()

        tform_new = TransformStamped()
        tform_new.header.frame_id = 'foo'
        tform_new.child_frame_id = 'bar'
        tform_new.transform.translation.x = 2.0
        tform_new.transform.translation.y = 3.0
        tform_new.transform.translation.z = 4.0
        tform_new.transform.rotation.w = 1.0
        tform_new.transform.rotation.x = 0.0
        tform_new.transform.rotation.y = 0.0
        tform_new.transform.rotation.z = 0.0

        req.transform = tform_new
        future = tform_client.call_async(req)

        rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)

        n = 0
        while n < 20:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            n += 1

        tform_updated = self.buffer.lookup_transform('foo', 'bar', Time(seconds=0), timeout=Duration(seconds=5))

        # make sure that the transform received matches the new value we set
        assert tform_updated.header.frame_id == 'foo'
        assert tform_updated.child_frame_id == 'bar'
        assert tform_updated.transform.translation.x == tform_new.transform.translation.x
        assert tform_updated.transform.translation.y == tform_new.transform.translation.y
        assert tform_updated.transform.translation.z == tform_new.transform.translation.z
        assert tform_updated.transform.rotation.w == tform_new.transform.rotation.w
        assert tform_updated.transform.rotation.x == tform_new.transform.rotation.x
        assert tform_updated.transform.rotation.y == tform_new.transform.rotation.y
        assert tform_updated.transform.rotation.z == tform_new.transform.rotation.z

