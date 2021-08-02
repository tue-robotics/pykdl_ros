import unittest

from pykdl_ros import VectorStamped, FrameStamped
import rospy

import tf2_ros
# noinspection PyUnresolvedReferences
import tf2_pykdl_ros

from geometry_msgs.msg import PointStamped


class TestRegistration(unittest.TestCase):
    def test_vector_stamped_point_stamped(self):
        v = VectorStamped.from_xyz(1, 2, 3, rospy.Time(), "map")
        p = tf2_ros.convert(v, PointStamped)
        self.assertIsInstance(p, PointStamped)
        self.assertEqual(p.point.x, v.vector.x())
        self.assertEqual(p.point.y, v.vector.y())
        self.assertEqual(p.point.z, v.vector.z())
        self.assertEqual(p.header.stamp, v.header.stamp)
        self.assertEqual(p.header.frame_id, v.header.frame_id)

        # v = PyKDL.Vector(1,2,3)
        # out = b.transform(tf2_ros.Stamped(v, rospy.Time(2), 'a'), 'b')
        # self.assertEqual(out.x(), 0)
        # self.assertEqual(out.y(), -2)
        # self.assertEqual(out.z(), -3)
        #
        # f = PyKDL.Frame(PyKDL.Rotation.RPY(1,2,3), PyKDL.Vector(1,2,3))
        # out = b.transform(tf2_ros.Stamped(f, rospy.Time(2), 'a'), 'b')
        # print(out)
        # self.assertEqual(out.p.x(), 0)
        # self.assertEqual(out.p.y(), -2)
        # self.assertEqual(out.p.z(), -3)
        # # TODO(tfoote) check values of rotation
        #
        # t = PyKDL.Twist(PyKDL.Vector(1,2,3), PyKDL.Vector(4,5,6))
        # out = b.transform(tf2_ros.Stamped(t, rospy.Time(2), 'a'), 'b')
        # self.assertEqual(out.vel.x(), 1)
        # self.assertEqual(out.vel.y(), -8)
        # self.assertEqual(out.vel.z(), 2)
        # self.assertEqual(out.rot.x(), 4)
        # self.assertEqual(out.rot.y(), -5)
        # self.assertEqual(out.rot.z(), -6)
        #
        # w = PyKDL.Wrench(PyKDL.Vector(1,2,3), PyKDL.Vector(4,5,6))
        # out = b.transform(tf2_ros.Stamped(w, rospy.Time(2), 'a'), 'b')
        # self.assertEqual(out.force.x(), 1)
        # self.assertEqual(out.force.y(), -2)
        # self.assertEqual(out.force.z(), -3)
        # self.assertEqual(out.torque.x(), 4)
        # self.assertEqual(out.torque.y(), -8)
        # self.assertEqual(out.torque.z(), -4)

    # def test_convert(self):
    #     v = PyKDL.Vector(1,2,3)
    #     vs = tf2_ros.Stamped(v, rospy.Time(2), 'a')
    #     vs2 = tf2_ros.convert(vs, PyKDL.Vector)
    #     self.assertEqual(vs.x(), 1)
    #     self.assertEqual(vs.y(), 2)
    #     self.assertEqual(vs.z(), 3)
    #     self.assertEqual(vs2.x(), 1)
    #     self.assertEqual(vs2.y(), 2)
    #     self.assertEqual(vs2.z(), 3)


if __name__ == "__main__":
    unittest.main()
