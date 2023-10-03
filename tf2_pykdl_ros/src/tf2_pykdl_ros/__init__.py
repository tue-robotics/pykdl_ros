from typing import Union

import PyKDL as kdl
import tf2_ros
from geometry_msgs.msg import (
    PointStamped,
    PoseStamped,
    TransformStamped,
    TwistStamped as TwistStampedMsg,
    WrenchStamped as WrenchStampedMsg,
)
from pykdl_ros import FrameStamped, TwistStamped, VectorStamped, WrenchStamped


def transform_to_kdl(t: TransformStamped) -> kdl.Frame:
    """
    Convert a geometry_msgs Transform message to a PyKDL Frame.

    :param t: The Transform message to convert.
    :return: The converted PyKDL frame.
    """

    return kdl.Frame(
        kdl.Rotation.Quaternion(
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w
        ),
        kdl.Vector(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z),
    )


def to_msg_vector(vector: VectorStamped) -> PointStamped:
    """
    Convert a VectorStamped to a geometry_msgs PointStamped message.

    :param vector: The vector to convert.
    :return: The converted vector/point.
    """
    msg = PointStamped()
    msg.header = vector.header
    msg.point.x = vector.vector[0]
    msg.point.y = vector.vector[1]
    msg.point.z = vector.vector[2]
    return msg


tf2_ros.ConvertRegistration().add_convert((VectorStamped, PointStamped), to_msg_vector)
tf2_ros.ConvertRegistration().add_to_msg(VectorStamped, to_msg_vector)


def from_msg_vector(msg: Union[PointStamped, PoseStamped]) -> VectorStamped:
    """
    Convert a PointStamped/PoseStamped message to a stamped VectorStamped.

    :param msg: The PointStamped/PoseStamped message to convert.
    :return: The timestamped converted PyKDL vector.
    """
    if isinstance(msg, PointStamped):
        v = msg.point
    elif isinstance(msg, PoseStamped):
        v = msg.pose.position
    else:
        raise TypeError(f"msg should be PointStamped or PoseStamped, not '{type(msg)}'")
    return VectorStamped.from_xyz(v.x, v.y, v.z, msg.header.stamp, msg.header.frame_id)


tf2_ros.ConvertRegistration().add_convert((PointStamped, VectorStamped), from_msg_vector)
tf2_ros.ConvertRegistration().add_convert((PoseStamped, VectorStamped), from_msg_vector)
tf2_ros.ConvertRegistration().add_from_msg(VectorStamped, from_msg_vector)


def convert_vector(vector: VectorStamped):
    """
    Convert a stamped PyKDL Vector to a stamped PyKDL Vector.

    :param vector: The vector to convert.
    :return: The timestamped converted PyKDL vector.
    """
    return VectorStamped(kdl.Vector(vector.vector), vector.header.stamp, vector.header.frame_id)


tf2_ros.ConvertRegistration().add_convert((VectorStamped, VectorStamped), convert_vector)


def do_transform_vector(vector: VectorStamped, transform: TransformStamped) -> VectorStamped:
    """
    Apply a transform in the form of a geometry_msgs message to a VectorStamped.

    :param vector: The VectorStamped to transform.
    :param transform: The transform to apply.
    :return: The transformed vector.
    """
    assert transform.child_frame_id == vector.header.frame_id
    res_vector = transform_to_kdl(transform) * vector.vector
    return VectorStamped(res_vector, transform.header.stamp, transform.header.frame_id)


tf2_ros.TransformRegistration().add(VectorStamped, do_transform_vector)


def to_msg_frame(frame: FrameStamped) -> PoseStamped:
    """
    Convert a FrameStamped to a geometry_msgs PoseStamped message.

    :param frame: The frame to convert.
    :return: The converted Pose.
    """
    msg = PoseStamped()
    msg.header = frame.header
    v = frame.frame.p
    msg.pose.position.x = v[0]
    msg.pose.position.y = v[1]
    msg.pose.position.z = v[2]
    q = frame.frame.M.GetQuaternion()
    msg.pose.orientation.x = q[0]
    msg.pose.orientation.y = q[1]
    msg.pose.orientation.z = q[2]
    msg.pose.orientation.w = q[3]
    return msg


tf2_ros.ConvertRegistration().add_convert((FrameStamped, PoseStamped), to_msg_frame)
tf2_ros.ConvertRegistration().add_to_msg(FrameStamped, to_msg_frame)


def from_msg_frame(msg: PoseStamped) -> FrameStamped:
    """
    Convert a PoseStamped message to a stamped FrameStamped.

    :param msg: The PoseStamped message to convert.
    :return: The timestamped converted PyKDL frame.
    """
    if not isinstance(msg, PoseStamped):
        raise TypeError(f"msg should be PoseStamped, not '{type(msg)}'")
    v = msg.pose.position
    vector = kdl.Vector(v.x, v.y, v.z)
    q = msg.pose.orientation
    rotation = kdl.Rotation.Quaternion(q.x, q.y, q.z, q.w)
    frame = kdl.Frame(rotation, vector)
    return FrameStamped(frame, msg.header.stamp, msg.header.frame_id)


tf2_ros.ConvertRegistration().add_convert((PoseStamped, FrameStamped), from_msg_frame)
tf2_ros.ConvertRegistration().add_from_msg(FrameStamped, from_msg_frame)


def convert_frame(frame: FrameStamped) -> FrameStamped:
    """
    Convert a stamped PyKDL Frame to a stamped PyKDL Frame.

    :param frame: The frame to convert.
    :return: The timestamped converted PyKDL frame.
    """
    return FrameStamped(kdl.Frame(frame.frame), frame.header.stamp, frame.header.frame_id)


tf2_ros.ConvertRegistration().add_convert((FrameStamped, FrameStamped), convert_frame)


def do_transform_frame(frame: FrameStamped, transform: TransformStamped) -> FrameStamped:
    """
    Apply a transform in the form of a geometry_msgs message to a PyKDL Frame.

    :param frame: The PyKDL frame to transform.
    :param transform: The transform to apply.
    :return: The transformed PyKDL frame.
    """
    assert transform.child_frame_id == frame.header.frame_id
    res_frame = transform_to_kdl(transform) * frame.frame
    return FrameStamped(res_frame, transform.header.stamp, transform.header.frame_id)


tf2_ros.TransformRegistration().add(FrameStamped, do_transform_frame)


def to_msg_twist(twist: TwistStamped) -> TwistStampedMsg:
    """
    Convert a FrameStamped to a geometry_msgs TwistStamped message.

    :param twist: The frame to convert.
    :return: The converted Twist.
    """
    msg = TwistStampedMsg()
    msg.header = twist.header
    vel = twist.twist.vel
    msg.twist.linear.x = vel[0]
    msg.twist.linear.y = vel[1]
    msg.twist.linear.z = vel[2]
    rot = twist.twist.rot
    msg.twist.angular.x = rot[0]
    msg.twist.angular.y = rot[1]
    msg.twist.angular.z = rot[2]
    return msg


tf2_ros.ConvertRegistration().add_convert((TwistStamped, TwistStampedMsg), to_msg_twist)
tf2_ros.ConvertRegistration().add_to_msg(TwistStamped, to_msg_twist)


def from_msg_twist(msg: TwistStampedMsg) -> TwistStamped:
    """
    Convert a TwistStamped message to a stamped TwistStamped.

    :param msg: The TwistStamped message to convert.
    :return: The timestamped converted PyKDL vector.
    """
    if not isinstance(msg, TwistStampedMsg):
        raise TypeError(f"msg should be TwistStamped, not '{type(msg)}'")
    lin = msg.twist.linear
    vel = kdl.Vector(lin.x, lin.y, lin.z)
    ang = msg.twist.angular
    rot = kdl.Vector(ang.x, ang.y, ang.z)
    twist = kdl.Twist(vel, rot)
    return TwistStamped(twist, msg.header.stamp, msg.header.frame_id)


tf2_ros.ConvertRegistration().add_convert((TwistStampedMsg, TwistStamped), from_msg_twist)
tf2_ros.ConvertRegistration().add_from_msg(TwistStamped, from_msg_twist)


def convert_twist(twist: TwistStamped) -> TwistStamped:
    """
    Convert a stamped PyKDL Twist to a stamped PyKDL Twist.

    :param twist: The twist to convert.
    :return: The timestamped converted PyKDL twist.
    """
    return TwistStamped(kdl.Twist(twist.twist), twist.header.stamp, twist.header.frame_id)


tf2_ros.ConvertRegistration().add_convert((TwistStamped, TwistStamped), convert_twist)


def do_transform_twist(twist: TwistStamped, transform: TransformStamped) -> TwistStamped:
    """
    Apply a transform in the form of a geometry_msgs message to a PyKDL Frame.

    :param twist: The PyKDL twist to transform.
    :param transform: The transform to apply.
    :return: The transformed PyKDL twist.
    """
    assert transform.child_frame_id == twist.header.frame_id
    res_twist = transform_to_kdl(transform) * twist.twist
    return TwistStamped(res_twist, transform.header.stamp, transform.header.frame_id)


tf2_ros.TransformRegistration().add(TwistStamped, do_transform_twist)


def to_msg_wrench(wrench: WrenchStamped) -> WrenchStampedMsg:
    """
    Convert a FrameStamped to a geometry_msgs WrenchStamped message.

    :param wrench: The wrench to convert.
    :return: The converted Wrench.
    """
    msg = WrenchStampedMsg()
    msg.header = wrench.header
    force = wrench.wrench.force
    msg.wrench.force.x = force[0]
    msg.wrench.force.y = force[1]
    msg.wrench.force.z = force[2]
    torque = wrench.wrench.torque
    msg.wrench.torque.x = torque[0]
    msg.wrench.torque.y = torque[1]
    msg.wrench.torque.z = torque[2]
    return msg


tf2_ros.ConvertRegistration().add_convert((WrenchStamped, WrenchStampedMsg), to_msg_wrench)
tf2_ros.ConvertRegistration().add_to_msg(WrenchStamped, to_msg_wrench)


def from_msg_wrench(msg: WrenchStampedMsg) -> WrenchStamped:
    """
    Convert a WrenchStamped message to a stamped WrenchStamped.

    :param msg: The WrenchStamped message to convert.
    :return: The timestamped converted PyKDL wrench.
    """
    if not isinstance(msg, WrenchStampedMsg):
        raise TypeError(f"msg should be WrenchStamped, not '{type(msg)}'")
    f = msg.wrench.force
    t = msg.wrench.torque
    force = kdl.Vector(f.x, f.y, f.z)
    torque = kdl.Vector(t.x, t.y, t.z)
    wrench = kdl.Wrench(force, torque)
    return WrenchStamped(wrench, msg.header.stamp, msg.header.frame_id)


tf2_ros.ConvertRegistration().add_convert((WrenchStampedMsg, WrenchStamped), from_msg_wrench)
tf2_ros.ConvertRegistration().add_from_msg(WrenchStamped, from_msg_wrench)


def convert_wrench(wrench: WrenchStamped) -> WrenchStamped:
    """
    Convert a stamped PyKDL Wrench to a stamped PyKDL Wrench.

    :param wrench: The wrench to convert.
    :return: The timestamped converted PyKDL wrench.
    """
    return WrenchStamped(kdl.Wrench(wrench.wrench), wrench.header.stamp, wrench.header.frame_id)


tf2_ros.ConvertRegistration().add_convert((WrenchStamped, WrenchStamped), convert_wrench)


def do_transform_wrench(wrench: WrenchStamped, transform: TransformStamped) -> WrenchStamped:
    """
    Apply a transform in the form of a geometry_msgs message to a PyKDL Frame.

    :param wrench: The PyKDL wrench to transform.
    :param transform: The transform to apply.
    :return: The transformed PyKDL wrench.
    """
    assert transform.child_frame_id == wrench.header.frame_id
    res_wrench = transform_to_kdl(transform) * wrench.wrench
    return WrenchStamped(res_wrench, transform.header.stamp, transform.header.frame_id)


tf2_ros.TransformRegistration().add(WrenchStamped, do_transform_wrench)
