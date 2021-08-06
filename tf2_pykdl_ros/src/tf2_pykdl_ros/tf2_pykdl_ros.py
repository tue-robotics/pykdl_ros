import geometry_msgs.msg as gm
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


def to_msg_vector(vector: VectorStamped):
    """
    Convert a VectorStamped to a geometry_msgs PointStamped message.

    :param vector: The vector to convert.
    :type vector: pykdl_ros.VectorStamped
    :return: The converted vector/point.
    :rtype: geometry_msgs.msg.PointStamped
    """
    msg = gm.PointStamped()
    msg.header = vector.header
    msg.point.x = vector.vector[0]
    msg.point.y = vector.vector[1]
    msg.point.z = vector.vector[2]
    return msg


tf2_ros.ConvertRegistration().add_to_msg(VectorStamped, to_msg_vector)


def from_msg_vector(msg):
    """
    Convert a PointStamped message to a stamped VectorStamped.

    :param msg: The PointStamped message to convert.
    :type msg: geometry_msgs.msg.PointStamped
    :return: The timestamped converted PyKDL vector.
    :rtype: pykdl_ros.VectorStamped
    """
    vector = VectorStamped.from_xyz(msg.point.x, msg.point.y, msg.point.z, msg.header.stamp, msg.header.frame_id)
    return vector


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


def do_transform_vector(vector: VectorStamped, transform: gm.TransformStamped):
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


def do_transform_frame(frame: FrameStamped, transform: gm.TransformStamped):
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
