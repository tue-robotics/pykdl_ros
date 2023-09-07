import unittest

import PyKDL as kdl
import tf2_geometry_msgs  # noqa: F401
import tf2_ros
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from geometry_msgs.msg import TwistStamped as TwistStampedMsg
from geometry_msgs.msg import WrenchStamped as WrenchStampedMsg
from pykdl_ros import FrameStamped, TwistStamped, VectorStamped, WrenchStamped

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

    def test_twist_stamped_from_msg(self):
        self.convert_reg.get_from_msg(TwistStamped)

    def test_twist_stamp_to_msg(self):
        self.convert_reg.get_to_msg(TwistStamped)

    def test_twist_stamp_convert(self):
        self.convert_reg.get_convert((TwistStampedMsg, TwistStamped))
        self.convert_reg.get_convert((TwistStamped, TwistStampedMsg))
        self.convert_reg.get_convert((TwistStamped, TwistStamped))

    def test_twist_stamped_transform(self):
        self.transform_reg.get(TwistStamped)

    def test_wrench_stamped_from_msg(self):
        self.convert_reg.get_from_msg(WrenchStamped)

    def test_wrench_stamped_to_msg(self):
        self.convert_reg.get_to_msg(WrenchStamped)

    def test_wrench_stamped_convert(self):
        self.convert_reg.get_convert((WrenchStampedMsg, WrenchStamped))
        self.convert_reg.get_convert((WrenchStamped, WrenchStampedMsg))
        self.convert_reg.get_convert((WrenchStamped, WrenchStamped))

    def test_wrench_stamped_transform(self):
        self.transform_reg.get(WrenchStamped)


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

    def test_pose_stamp_frame_stamped(self):
        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = Time(sec=4)
        p.pose.position.x = 1.0
        p.pose.position.y = 2.0
        p.pose.position.z = 3.0
        p.pose.orientation.x = 0.0
        p.pose.orientation.y = 0.0
        p.pose.orientation.z = 0.0
        p.pose.orientation.w = 1.0
        f = tf2_ros.convert(p, FrameStamped)
        self.assertIsInstance(f, FrameStamped)
        self.assertEqual(f.frame.p.x(), p.pose.position.x)
        self.assertEqual(f.frame.p.y(), p.pose.position.y)
        self.assertEqual(f.frame.p.z(), p.pose.position.z)
        self.assertEqual(f.frame.M.GetQuaternion(), (0.0, 0.0, 0.0, 1.0))
        self.assertEqual(f.header.stamp, p.header.stamp)
        self.assertEqual(f.header.frame_id, p.header.frame_id)

    def test_frame_stamped_pose_stamped(self):
        f = FrameStamped.from_xyz_rpy(1, 2, 3, 4, 5, 6, Time(sec=4), "map")
        p = tf2_ros.convert(f, PoseStamped)
        self.assertIsInstance(p, PoseStamped)
        self.assertEqual(p.pose.position.x, f.frame.p.x())
        self.assertEqual(p.pose.position.y, f.frame.p.y())
        self.assertEqual(p.pose.position.z, f.frame.p.z())
        q = f.frame.M.GetQuaternion()
        self.assertEqual(p.pose.orientation.x, q[0])
        self.assertEqual(p.pose.orientation.y, q[1])
        self.assertEqual(p.pose.orientation.z, q[2])
        self.assertEqual(p.pose.orientation.w, q[3])
        self.assertEqual(p.header.stamp, f.header.stamp)
        self.assertEqual(p.header.frame_id, f.header.frame_id)

    def test_twist_stamped_msg_twist_stamped(self):
        msg = TwistStampedMsg()
        msg.header.frame_id = "map"
        msg.header.stamp = Time(sec=4)
        msg.twist.linear.x = 1.0
        msg.twist.linear.y = 2.0
        msg.twist.linear.z = 3.0
        msg.twist.angular.x = 4.0
        msg.twist.angular.y = 5.0
        msg.twist.angular.z = 6.0
        t = tf2_ros.convert(msg, TwistStamped)
        self.assertIsInstance(t, TwistStamped)
        self.assertEqual(t.twist.vel.x(), msg.twist.linear.x)
        self.assertEqual(t.twist.vel.y(), msg.twist.linear.y)
        self.assertEqual(t.twist.vel.z(), msg.twist.linear.z)
        self.assertEqual(t.twist.rot.x(), msg.twist.angular.x)
        self.assertEqual(t.twist.rot.y(), msg.twist.angular.y)
        self.assertEqual(t.twist.rot.z(), msg.twist.angular.z)
        self.assertEqual(t.header.stamp, msg.header.stamp)
        self.assertEqual(t.header.frame_id, msg.header.frame_id)

    def test_twist_stamped_twist_stamped_msg(self):
        t = TwistStamped.from_xyz_rpy(1, 2, 3, 4, 5, 6, Time(sec=4), "map")
        msg = tf2_ros.convert(t, TwistStampedMsg)
        self.assertIsInstance(msg, TwistStampedMsg)
        self.assertEqual(msg.twist.linear.x, t.twist.vel.x())
        self.assertEqual(msg.twist.linear.y, t.twist.vel.y())
        self.assertEqual(msg.twist.linear.z, t.twist.vel.z())
        self.assertEqual(msg.twist.angular.x, t.twist.rot.x())
        self.assertEqual(msg.twist.angular.y, t.twist.rot.y())
        self.assertEqual(msg.twist.angular.z, t.twist.rot.z())
        self.assertEqual(msg.header.stamp, t.header.stamp)
        self.assertEqual(msg.header.frame_id, t.header.frame_id)

    def test_wrench_stamped_msg_wrench_stamped(self):
        msg = WrenchStampedMsg()
        msg.header.frame_id = "map"
        msg.header.stamp = Time(sec=4)
        msg.wrench.force.x = 1.0
        msg.wrench.force.y = 2.0
        msg.wrench.force.z = 3.0
        msg.wrench.torque.x = 4.0
        msg.wrench.torque.y = 5.0
        msg.wrench.torque.z = 6.0
        w = tf2_ros.convert(msg, WrenchStamped)
        self.assertIsInstance(w, WrenchStamped)
        self.assertEqual(w.wrench.force.x(), msg.wrench.force.x)
        self.assertEqual(w.wrench.force.y(), msg.wrench.force.y)
        self.assertEqual(w.wrench.force.z(), msg.wrench.force.z)
        self.assertEqual(w.wrench.torque.x(), msg.wrench.torque.x)
        self.assertEqual(w.wrench.torque.y(), msg.wrench.torque.y)
        self.assertEqual(w.wrench.torque.z(), msg.wrench.torque.z)
        self.assertEqual(w.header.stamp, msg.header.stamp)
        self.assertEqual(w.header.frame_id, msg.header.frame_id)

    def test_wrench_stamped_wrench_stamped_msg(self):
        w = WrenchStamped.from_fxfyfz_txtytz(1, 2, 3, 4, 5, 6, Time(sec=4), "map")
        msg = tf2_ros.convert(w, WrenchStampedMsg)
        self.assertIsInstance(msg, WrenchStampedMsg)
        self.assertEqual(msg.wrench.force.x, w.wrench.force.x())
        self.assertEqual(msg.wrench.force.y, w.wrench.force.y())
        self.assertEqual(msg.wrench.force.z, w.wrench.force.z())
        self.assertEqual(msg.wrench.torque.x, w.wrench.torque.x())
        self.assertEqual(msg.wrench.torque.y, w.wrench.torque.y())
        self.assertEqual(msg.wrench.torque.z, w.wrench.torque.z())
        self.assertEqual(msg.header.stamp, w.header.stamp)
        self.assertEqual(msg.header.frame_id, w.header.frame_id)


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
