#! /usr/bin/env python

import unittest
from typing import ClassVar

import PyKDL as kdl
import rclpy
import tf2_geometry_msgs  # noqa: F401
import tf2_ros
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from pykdl_ros import FrameStamped, TwistStamped, VectorStamped, WrenchStamped
from rclpy.duration import Duration

import tf2_pykdl_ros  # noqa: F401


class TestTransform(unittest.TestCase):
    buffer: ClassVar[tf2_ros.Buffer]
    context: ClassVar[rclpy.context.Context]
    node: ClassVar[rclpy.node.Node]

    @classmethod
    def setUpClass(cls) -> None:
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)

        cls.node = rclpy.create_node(node_name="test_tf2_ros_transform", context=cls.context)
        cls.buffer = tf2_ros.Buffer()
        t = TransformStamped()
        t.transform.translation.x = 1.0
        t.transform.rotation.x = 1.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.0
        t.header.stamp = Time(sec=2)
        t.header.frame_id = "a"
        t.child_frame_id = "b"
        cls.buffer.set_transform(t, "test_tf2_ros_convert")

    @classmethod
    def tearDownClass(cls) -> None:
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def test_lookup_transform(self) -> None:
        out = self.buffer.lookup_transform("a", "b", Time(sec=2), Duration(seconds=2))
        self.assertEqual(out.transform.translation.x, 1)
        self.assertEqual(out.transform.rotation.x, 1)
        self.assertEqual(out.header.frame_id, "a")
        self.assertEqual(out.child_frame_id, "b")

    def test_transform_vector(self) -> None:
        v = kdl.Vector(1, 2, 3)
        out: VectorStamped = self.buffer.transform(VectorStamped(v, Time(sec=2), "a"), "b")
        self.assertEqual(out.vector.x(), 0)
        self.assertEqual(out.vector.y(), -2)
        self.assertEqual(out.vector.z(), -3)

    def test_transform_frame(self) -> None:
        f = kdl.Frame(kdl.Rotation.RPY(1, 2, 3), kdl.Vector(1, 2, 3))
        out: FrameStamped = self.buffer.transform(FrameStamped(f, Time(sec=2), "a"), "b")
        self.assertEqual(out.frame.p.x(), 0)
        self.assertEqual(out.frame.p.y(), -2)
        self.assertEqual(out.frame.p.z(), -3)
        self.assertEqual(
            out.frame.M.GetQuaternion(),
            (0.43595284407356577, -0.44443511344300074, 0.310622451065704, 0.7182870182434113),
        )

    def test_transform_twist(self) -> None:
        t = kdl.Twist(kdl.Vector(1, 2, 3), kdl.Vector(4, 5, 6))
        out: TwistStamped = self.buffer.transform(TwistStamped(t, Time(sec=2), "a"), "b")
        self.assertEqual(out.twist.vel.x(), 1)
        self.assertEqual(out.twist.vel.y(), -8)
        self.assertEqual(out.twist.vel.z(), 2)
        self.assertEqual(out.twist.rot.x(), 4)
        self.assertEqual(out.twist.rot.y(), -5)
        self.assertEqual(out.twist.rot.z(), -6)

    def test_transform_wrench(self) -> None:
        w = kdl.Wrench(kdl.Vector(1, 2, 3), kdl.Vector(4, 5, 6))
        out: WrenchStamped = self.buffer.transform(WrenchStamped(w, Time(sec=2), "a"), "b")
        self.assertEqual(out.wrench.force.x(), 1)
        self.assertEqual(out.wrench.force.y(), -2)
        self.assertEqual(out.wrench.force.z(), -3)
        self.assertEqual(out.wrench.torque.x(), 4)
        self.assertEqual(out.wrench.torque.y(), -8)
        self.assertEqual(out.wrench.torque.z(), -4)
