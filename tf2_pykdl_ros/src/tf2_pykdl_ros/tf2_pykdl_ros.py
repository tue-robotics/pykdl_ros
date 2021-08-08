from typing import Union
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
import PyKDL as kdl
from pykdl_ros import VectorStamped, FrameStamped
import tf2_ros


def transform_to_kdl(t):
    """
    Convert a geometry_msgs Transform message to a PyKDL Frame.

    :param t: The Transform message to convert.
    :type t: geometry_msgs.msg.TransformStamped
    :return: The converted PyKDL frame.
    :rtype: PyKDL.Frame
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
    :type vector: pykdl_ros.VectorStamped
    :return: The converted vector/point.
    :rtype: geometry_msgs.msg.PointStamped
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
    Convert a PointStamped message to a stamped VectorStamped.

    :param msg: The PointStamped/PoseStamped message to convert.
    :type msg: Union[geometry_msgs.msg.PointStamped, geometry.msg.PoseStamped]
    :return: The timestamped converted PyKDL vector.
    :rtype: pykdl_ros.VectorStamped
    """
    if isinstance(msg, PointStamped):
        v = msg.point
    elif isinstance(msg, PoseStamped):
        v = msg.pose.position
    else:
        raise TypeError(f"msg should be PointStamped or PoseStamped, not '{type(msg)}'")
    vector = VectorStamped.from_xyz(v.x, v.y, v.z, msg.header.stamp, msg.header.frame_id)
    return vector


tf2_ros.ConvertRegistration().add_convert((PointStamped, VectorStamped), from_msg_vector)
tf2_ros.ConvertRegistration().add_convert((PoseStamped, VectorStamped), from_msg_vector)
tf2_ros.ConvertRegistration().add_from_msg(VectorStamped, from_msg_vector)


def convert_vector(vector):
    """
    Convert a generic stamped triplet message to a stamped PyKDL Vector.

    :param vector: The message to convert.
    :type vector: pykdl_ros.VectorStamped
    :return: The timestamped converted PyKDL vector.
    :rtype: pykdl_ros.VectorStamped
    """
    return VectorStamped(vector.vector, vector.header.stamp, vector.header.frame_id)


tf2_ros.ConvertRegistration().add_convert((VectorStamped, VectorStamped), convert_vector)


def do_transform_vector(vector: VectorStamped, transform: TransformStamped) -> VectorStamped:
    """
    Apply a transform in the form of a geometry_msgs message to a VectorStamped.

    :param vector: The VectorStamped to transform.
    :type vector: pykdl_ros.VectorStamped
    :param transform: The transform to apply.
    :type transform: geometry_msgs.msg.TransformStamped
    :return: The transformed vector.
    :rtype: pykdl_ros.VectorStamped
    """
    assert transform.child_frame_id == vector.header.frame_id
    res_vector = transform_to_kdl(transform) * vector.vector
    res = VectorStamped(res_vector, transform.header.stamp, transform.header.frame_id)
    return res


tf2_ros.TransformRegistration().add(VectorStamped, do_transform_vector)


def to_msg_frame(frame: FrameStamped) -> PoseStamped:
    """
    Convert a FrameStamped to a geometry_msgs PoseStamped message.

    :param frame: The frame to convert.
    :type frame: pykdl_ros.FrameStamped
    :return: The converted Pose.
    :rtype: geometry_msgs.msg.PointStamped
    """
    msg = PoseStamped()
    msg.header = frame.header
    v = frame.frame.p
    msg.pose.position.x = v[0]
    msg.pose.position.y = v[1]
    msg.pose.position.z = v[2]
    q = frame.frame.M.GetQuaternion()
    msg.pose.orientation.x = q.x
    msg.pose.orientation.y = q.y
    msg.pose.orientation.z = q.z
    msg.pose.orientation.w = q.w
    return msg


tf2_ros.ConvertRegistration().add_convert((FrameStamped, PoseStamped), to_msg_frame)
tf2_ros.ConvertRegistration().add_to_msg(FrameStamped, to_msg_frame)


def from_msg_frame(msg: PoseStamped) -> FrameStamped:
    """
    Convert a PoseStamped message to a stamped FrameStamped.

    :param msg: The PoseStamped message to convert.
    :type msg: geometry.msg.PoseStamped
    :return: The timestamped converted PyKDL vector.
    :rtype: pykdl_ros.FrameStamped
    """
    if not isinstance(msg, PoseStamped):
        raise TypeError(f"msg should be PoseStamped, not '{type(msg)}'")
    v = msg.pose.position
    vector = kdl.Vector(v.x, v.y, v.z)
    q = msg.pose.orientation
    rotation = kdl.Rotation.Quaternion(q.x, q.y, q.z, q.w)
    frame = kdl.Frame(rotation, vector)
    frame_stamped = FrameStamped(frame, msg.header.stamp, msg.header.frame_id)
    return frame_stamped


tf2_ros.ConvertRegistration().add_convert((PoseStamped, FrameStamped), from_msg_frame)
tf2_ros.ConvertRegistration().add_from_msg(FrameStamped, from_msg_frame)


def do_transform_frame(frame: FrameStamped, transform: TransformStamped) -> TransformStamped:
    """
    Apply a transform in the form of a geometry_msgs message to a PyKDL Frame.

    :param frame: The PyKDL frame to transform.
    :type frame: pykdl_ros.FrameStamped
    :param transform: The transform to apply.
    :type transform: geometry_msgs.msg.TransformStamped
    :return: The transformed PyKDL frame.
    :rtype: PyKDL.Frame
    """
    assert transform.child_frame_id == frame.header.frame_id
    res_frame = transform_to_kdl(transform) * frame.frame
    res = FrameStamped(res_frame, transform.header.stamp, transform.header.frame_id)
    return res


tf2_ros.TransformRegistration().add(FrameStamped, do_transform_frame)
