#! /usr/bin/env python

import unittest

import PyKDL as kdl
from pykdl_ros import VectorStamped, FrameStamped, TwistStamped, WrenchStamped
import rospy

import tf2_ros

# noinspection PyUnresolvedReferences
import tf2_geometry_msgs

# noinspection PyUnresolvedReferences
import tf2_pykdl_ros

from geometry_msgs.msg import TransformStamped


class TestTransform(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.buffer = tf2_ros.Buffer()
        t = TransformStamped()
        t.transform.translation.x = 1.0
        t.transform.rotation.x = 1.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.0
        t.header.stamp = rospy.Time(2)
        t.header.frame_id = "a"
        t.child_frame_id = "b"
        cls.buffer.set_transform(t, "test_tf2_ros_convert")

    def test_lookup_transform(self):
        out = self.buffer.lookup_transform("a", "b", rospy.Time(2), rospy.Duration(2))
        self.assertEqual(out.transform.translation.x, 1)
        self.assertEqual(out.transform.rotation.x, 1)
        self.assertEqual(out.header.frame_id, "a")
        self.assertEqual(out.child_frame_id, "b")

    def test_transform_vector(self):
        v = kdl.Vector(1, 2, 3)
        out: VectorStamped = self.buffer.transform(VectorStamped(v, rospy.Time(2), "a"), "b")
        self.assertEqual(out.vector.x(), 0)
        self.assertEqual(out.vector.y(), -2)
        self.assertEqual(out.vector.z(), -3)

    def test_transform_frame(self):
        f = kdl.Frame(kdl.Rotation.RPY(1, 2, 3), kdl.Vector(1, 2, 3))
        out: FrameStamped = self.buffer.transform(FrameStamped(f, rospy.Time(2), "a"), "b")
        self.assertEqual(out.frame.p.x(), 0)
        self.assertEqual(out.frame.p.y(), -2)
        self.assertEqual(out.frame.p.z(), -3)
        self.assertEqual(
            out.frame.M.GetQuaternion(),
            (0.43595284407356577, -0.44443511344300074, 0.310622451065704, 0.7182870182434113),
        )

    def test_transform_twist(self):
        t = kdl.Twist(kdl.Vector(1, 2, 3), kdl.Vector(4, 5, 6))
        out: TwistStamped = self.buffer.transform(TwistStamped(t, rospy.Time(2), "a"), "b")
        self.assertEqual(out.twist.vel.x(), 1)
        self.assertEqual(out.twist.vel.y(), -8)
        self.assertEqual(out.twist.vel.z(), 2)
        self.assertEqual(out.twist.rot.x(), 4)
        self.assertEqual(out.twist.rot.y(), -5)
        self.assertEqual(out.twist.rot.z(), -6)

    def test_transform_wrench(self):
        w = kdl.Wrench(kdl.Vector(1, 2, 3), kdl.Vector(4, 5, 6))
        out: WrenchStamped = self.buffer.transform(WrenchStamped(w, rospy.Time(2), "a"), "b")
        self.assertEqual(out.wrench.force.x(), 1)
        self.assertEqual(out.wrench.force.y(), -2)
        self.assertEqual(out.wrench.force.z(), -3)
        self.assertEqual(out.wrench.torque.x(), 4)
        self.assertEqual(out.wrench.torque.y(), -8)
        self.assertEqual(out.wrench.torque.z(), -4)


if __name__ == "__main__":
    import rosunit

    rospy.init_node("test_tf2_pykdl_ros")
    rosunit.unitrun("test_tf2_pykdl_ros", "test_tf2_ros_convert", TestTransform)
