import unittest

from pykdl_ros import VectorStamped, FrameStamped
import rospy

import tf2_ros

# noinspection PyUnresolvedReferences
import tf2_geometry_msgs

# noinspection PyUnresolvedReferences
import tf2_kdl

# noinspection PyUnresolvedReferences
import tf2_pykdl_ros

from geometry_msgs.msg import PointStamped


class TestRegistration(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.convert_reg = tf2_ros.ConvertRegistration()
        cls.transform_reg = tf2_ros.TransformRegistration()

    def test_vector_stamped_from_msg(self):
        self.convert_reg.get_from_msg(VectorStamped)

    def test_vector_stamped_to_msg(self):
        self.convert_reg.get_to_msg(VectorStamped)

    def test_vector_stamped_convert(self):
        self.convert_reg.get_convert((VectorStamped, VectorStamped))

    def test_vector_stamped_transform(self):
        self.transform_reg.get(VectorStamped)

    def test_frame_stamped_transform(self):
        self.transform_reg.get(FrameStamped)


class TestConvert(unittest.TestCase):
    def test_point_stamped_vector_stamped(self):
        p = PointStamped()
        p.header.frame_id = "map"
        p.header.stamp = rospy.Time(4)
        p.point.x = 1
        p.point.y = 2
        p.point.z = 3
        v = tf2_ros.convert(p, VectorStamped)
        self.assertIsInstance(v, VectorStamped)
        self.assertEqual(v.vector.x(), p.point.x)
        self.assertEqual(v.vector.y(), p.point.y)
        self.assertEqual(v.vector.z(), p.point.z)
        self.assertEqual(v.header.stamp, p.header.stamp)
        self.assertEqual(v.header.frame_id, p.header.frame_id)

    def test_vector_stamped_point_stamped(self):
        v = VectorStamped.from_xyz(1, 2, 3, rospy.Time(4), "map")
        p = tf2_ros.convert(v, PointStamped)
        self.assertIsInstance(p, PointStamped)
        self.assertEqual(p.point.x, v.vector.x())
        self.assertEqual(p.point.y, v.vector.y())
        self.assertEqual(p.point.z, v.vector.z())
        self.assertEqual(p.header.stamp, v.header.stamp)
        self.assertEqual(p.header.frame_id, v.header.frame_id)


if __name__ == "__main__":
    unittest.main()
