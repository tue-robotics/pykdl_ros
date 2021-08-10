from rospy import Time
import PyKDL as kdl
from std_msgs.msg import Header


class VectorStamped:
    """
    Stamped version of PyKDL.Vector
    """

    def __init__(self, vector: kdl.Vector, stamp: Time, frame_id: str):
        """
        Constructor

        :param vector: vector
        :type vector: PykDL.Vector
        :param stamp: TimeStamp
        :type stamp: rospy.Time
        :param frame_id: Frame ID
        :type frame_id: str
        """
        assert isinstance(vector, kdl.Vector)
        assert isinstance(stamp, Time)
        assert isinstance(frame_id, str)
        self.vector = vector
        self.header = Header(frame_id=frame_id, stamp=stamp)

    def __repr__(self):
        xyz = "(x={x}, y={y}, z={z})".format(x=self.vector.x(), y=self.vector.y(), z=self.vector.z())
        return "VectorStamped({xyz} @ {fid})".format(xyz=self.vector, fid=self.header.frame_id)

    def __eq__(self, other):
        if isinstance(other, VectorStamped):
            return self.vector == other.vector and self.header.frame_id == other.header.frame_id
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    @staticmethod
    def from_xyz(x: float, y: float, z: float, stamp: Time, frame_id: str):
        """
        Custom constructor

        :param x: x
        :type x: float
        :param y: y
        :type y: float
        :param z: z
        :type z: float
        :param stamp: TimeStamp
        :type stamp: rospy.Time
        :param frame_id: Frame ID
        :type frame_id: str
        :return: Filled object
        :rtype: VectorStamped
        """
        vector = kdl.Vector(x, y, z)
        return VectorStamped(vector, stamp, frame_id)

    @staticmethod
    def from_FrameStamped(frame: FrameStamped):
        """
        Custom constructor, extract vector from the frame

        :param frame: frame
        :type frame: FrameStamped
        :return: Filled object
        :rtype: VectorStamped
        """
        return VectorStamped(frame.frame.p, frame.header.stamp, frame.header.frame_id)


class FrameStamped:
    """
    Stamped version of PyKDL.Frame
    """

    def __init__(self, frame: kdl.Frame, stamp: Time, frame_id: str):
        """
        Constructor

        :param frame: frame
        :type frame: PykDL.Frame
        :param stamp: TimeStamp
        :type stamp: rospy.Time
        :param frame_id: Frame ID
        :type frame_id: str
        """
        assert isinstance(frame, kdl.Frame)
        assert isinstance(stamp, Time)
        assert isinstance(frame_id, str)
        self.frame = frame
        self.header = Header(frame_id=frame_id, stamp=stamp)

    def __repr__(self):
        xyz = "(x={x}, y={y}, z={z})".format(x=self.frame.p.x(), y=self.frame.p.y(), z=self.frame.p.z())
        r, p, y = self.frame.M.GetRPY()
        rpy = "(r={x}, p={y}, y={z})".format(x=r, y=p, z=y)
        return "FrameStamped(pos:{pos}, rot:{rot} @ {fid})".format(pos=xyz, rot=rpy, fid=self.header.frame_id)

    def __eq__(self, other):
        if isinstance(other, FrameStamped):
            return self.frame == other.frame and self.header.frame_id == other.header.frame_id
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    @staticmethod
    def from_xyz_RPY(x: float, y: float, z: float, R: float, P: float, Y: float, stamp: Time, frame_id: str):
        """
        Custom constructor

        :param x: x
        :type x: float
        :param y: y
        :type y: float
        :param z: z
        :type z: float
        :param R: Roll
        :type R: float
        :param P: Pitch
        :type P: float
        :param Y: Yaw
        :type Y: float
        :param stamp: TimeStamp
        :type stamp: rospy.Time
        :param frame_id: Frame ID
        :type frame_id: str
        :return: Filled object
        :rtype: VectorStamped
        """
        vector = kdl.Vector(x, y, z)
        rotation = kdl.Rotation(R, P, Y)
        frame = kdl.Frame(rotation, vector)
        return FrameStamped(frame, stamp, frame_id)
