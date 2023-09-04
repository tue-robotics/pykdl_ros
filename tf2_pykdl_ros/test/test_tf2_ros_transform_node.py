#! /usr/bin/env python

import unittest

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
import PyKDL as kdl
from pykdl_ros import FrameStamped
import rclpy
from rclpy.duration import Duration
import tf2_ros
import tf2_geometry_msgs  # noqa: F401
import tf2_pykdl_ros  # noqa: F401


class TestTransform(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown(context=cls.context)

    def setUp(self) -> None:
        self.node = rclpy.create_node(node_name="test_tf2_ros_transform", context=self.context)

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_transform(self):
        buffer = tf2_ros.Buffer()
        t = TransformStamped()
        t.transform.translation.x = 1.0
        t.transform.rotation.x = 1.0
        t.transform.rotation.w = 0.0
        t.header.stamp = Time(sec=2)
        t.header.frame_id = "a"
        t.child_frame_id = "b"
        buffer.set_transform(t, "test_tf2_ros_convert")
        out = buffer.lookup_transform("a", "b", Time(sec=2), Duration(seconds=2))
        self.assertEqual(out.transform.translation.x, 1)
        self.assertEqual(out.transform.rotation.x, 1)
        self.assertEqual(out.header.frame_id, "a")
        self.assertEqual(out.child_frame_id, "b")

        f = FrameStamped(kdl.Frame(kdl.Rotation.RPY(1, 2, 3), kdl.Vector(1, 2, 3)), Time(sec=2), "a")
        out = buffer.transform(f, "b")
        self.assertEqual(out.frame.p.x(), 0)
        self.assertEqual(out.frame.p.y(), -2)
        self.assertEqual(out.frame.p.z(), -3)
        self.assertEqual(
            out.frame.M.GetQuaternion(),
            (0.43595284407356577, -0.44443511344300074, 0.310622451065704, 0.7182870182434113),
        )
