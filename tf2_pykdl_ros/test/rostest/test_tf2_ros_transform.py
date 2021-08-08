#! /usr/bin/env python

import unittest

import PyKDL as kdl
from pykdl_ros import FrameStamped
import rospy

import tf2_ros

# noinspection PyUnresolvedReferences
import tf2_geometry_msgs

# noinspection PyUnresolvedReferences
import tf2_pykdl_ros

from geometry_msgs.msg import TransformStamped


class TestTransform(unittest.TestCase):
    def test_transform(self):
        buffer = tf2_ros.Buffer()
        t = TransformStamped()
        t.transform.translation.x = 1
        t.transform.rotation.x = 1
        t.header.stamp = rospy.Time(2)
        t.header.frame_id = "a"
        t.child_frame_id = "b"
        buffer.set_transform(t, "test_tf2_ros_convert")
        out = buffer.lookup_transform("a", "b", rospy.Time(2), rospy.Duration(2))
        self.assertEqual(out.transform.translation.x, 1)
        self.assertEqual(out.transform.rotation.x, 1)
        self.assertEqual(out.header.frame_id, "a")
        self.assertEqual(out.child_frame_id, "b")

        f = FrameStamped(kdl.Frame(kdl.Rotation.RPY(1, 2, 3), kdl.Vector(1, 2, 3)), rospy.Time(2), "a")
        out = buffer.transform(f, "b")
        self.assertEqual(out.frame.p.x(), 0)
        self.assertEqual(out.frame.p.y(), -2)
        self.assertEqual(out.frame.p.z(), -3)
        self.assertEqual(
            out.frame.M.GetQuaternion(),
            (0.43595284407356577, -0.44443511344300074, 0.310622451065704, 0.7182870182434113),
        )


if __name__ == "__main__":
    import rosunit

    rospy.init_node("test_tf2_pykdl_ros")
    rosunit.unitrun("test_tf2_pykdl_ros", "test_tf2_ros_convert", TestTransform)
