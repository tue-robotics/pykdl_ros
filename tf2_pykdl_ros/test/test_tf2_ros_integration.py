import unittest

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
import PyKDL as kdl
from pykdl_ros import VectorStamped, FrameStamped
import tf2_ros
import tf2_geometry_msgs  # noqa: F401
import tf2_pykdl_ros  # noqa: F401


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
        self.convert_reg.get_convert((PointStamped, VectorStamped))
        self.convert_reg.get_convert((PoseStamped, VectorStamped))
        self.convert_reg.get_convert((VectorStamped, PointStamped))
        self.convert_reg.get_convert((VectorStamped, VectorStamped))

    def test_vector_stamped_transform(self):
        self.transform_reg.get(VectorStamped)

    def test_frame_stamped_from_msg(self):
        self.convert_reg.get_from_msg(FrameStamped)

    def test_frame_stamped_to_msg(self):
        self.convert_reg.get_to_msg(FrameStamped)

    def test_frame_stamped_convert(self):
        self.convert_reg.get_convert((PoseStamped, FrameStamped))
        self.convert_reg.get_convert((FrameStamped, PoseStamped))
        self.convert_reg.get_convert((FrameStamped, FrameStamped))

    def test_frame_stamped_transform(self):
        self.transform_reg.get(FrameStamped)


class TestConvert(unittest.TestCase):
    def test_point_stamped_vector_stamped(self):
        p = PointStamped()
        p.header.frame_id = "map"
        p.header.stamp = Time(sec=4)
        p.point.x = 1.0
        p.point.y = 2.0
        p.point.z = 3.0
        v = tf2_ros.convert(p, VectorStamped)
        self.assertIsInstance(v, VectorStamped)
        self.assertEqual(v.vector.x(), p.point.x)
        self.assertEqual(v.vector.y(), p.point.y)
        self.assertEqual(v.vector.z(), p.point.z)
        self.assertEqual(v.header.stamp, p.header.stamp)
        self.assertEqual(v.header.frame_id, p.header.frame_id)

    def test_vector_stamped_point_stamped(self):
        v = VectorStamped.from_xyz(1, 2, 3, Time(sec=4), "map")
        p = tf2_ros.convert(v, PointStamped)
        self.assertIsInstance(p, PointStamped)
        self.assertEqual(p.point.x, v.vector.x())
        self.assertEqual(p.point.y, v.vector.y())
        self.assertEqual(p.point.z, v.vector.z())
        self.assertEqual(p.header.stamp, v.header.stamp)
        self.assertEqual(p.header.frame_id, v.header.frame_id)


class TestTransform(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.registration = tf2_ros.TransformRegistration()
        cls.t = TransformStamped()
        cls.t.transform.translation.x = -1.0
        cls.t.transform.rotation.x = -1.0
        cls.t.transform.rotation.w = 0.0
        cls.t.header.stamp = Time(sec=2)
        cls.t.header.frame_id = "a"
        cls.t.child_frame_id = "b"

    def test_transform(self):
        f = FrameStamped(kdl.Frame(kdl.Rotation.RPY(1, 2, 3), kdl.Vector(1, 2, 3)), Time(sec=2), "b")
        transform_func = self.registration.get(type(f))
        f_b = transform_func(f, self.t)
        self.assertEqual(f_b.frame.p.x(), 0)
        self.assertEqual(f_b.frame.p.y(), -2)
        self.assertEqual(f_b.frame.p.z(), -3)
        self.assertEqual(
            f_b.frame.M.GetQuaternion(),
            (0.43595284407356577, -0.44443511344300074, 0.310622451065704, 0.7182870182434113),
        )

    def test_transform_incorrect_frame(self):
        f2 = FrameStamped(kdl.Frame(kdl.Rotation.RPY(1, 2, 3), kdl.Vector(1, 2, 3)), Time(sec=2), "a")
        transform_func = self.registration.get(type(f2))
        with self.assertRaises(AssertionError):
            transform_func(f2, self.t)


if __name__ == "__main__":
    unittest.main()
