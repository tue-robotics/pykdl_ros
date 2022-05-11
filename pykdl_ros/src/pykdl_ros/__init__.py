from __future__ import annotations

from genpy import Time
import PyKDL as kdl
from std_msgs.msg import Header


class FrameStamped:
    """
    Stamped version of PyKDL.Frame
    """

    __slots__ = "frame", "header"

    def __init__(self, frame: kdl.Frame, stamp: Time, frame_id: str):
        """
        Constructor

        :param frame: frame
        :param stamp: TimeStamp
        :param frame_id: Frame ID
        """
        assert isinstance(frame, kdl.Frame)
        assert isinstance(stamp, Time)
        assert isinstance(frame_id, str)
        self.frame = frame
        self.header = Header(frame_id=frame_id, stamp=stamp)

    def __repr__(self):
        xyz = f"(x={self.frame.p.x()}, y={self.frame.p.y()}, z={self.frame.p.z()})"
        r, p, y = self.frame.M.GetRPY()
        rpy = f"(r={r}, p={p}, y={y})"
        return f"FrameStamped(pos:{xyz}, rot:{rpy} @ {self.header.frame_id})"

    def __eq__(self, other):
        if isinstance(other, FrameStamped):
            return self.frame == other.frame and self.header.frame_id == other.header.frame_id
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash((self.frame, self.header.stamp, self.header.frame_id))

    @classmethod
    def from_xyz_rpy(
        cls, x: float, y: float, z: float, roll: float, pitch: float, yaw: float, stamp: Time, frame_id: str
    ) -> FrameStamped:
        """
        Custom constructor

        :param x: x
        :param y: y
        :param z: z
        :param roll: Roll
        :param pitch: Pitch
        :param yaw: Yaw
        :param stamp: TimeStamp
        :param frame_id: Frame ID
        :return: Filled object
        """
        vector = kdl.Vector(x, y, z)
        rotation = kdl.Rotation.RPY(roll, pitch, yaw)
        frame = kdl.Frame(rotation, vector)
        return cls(frame, stamp, frame_id)


class VectorStamped:
    """
    Stamped version of PyKDL.Vector
    """

    __slots__ = "vector", "header"

    def __init__(self, vector: kdl.Vector, stamp: Time, frame_id: str):
        """
        Constructor

        :param vector: vector
        :param stamp: TimeStamp
        :param frame_id: Frame ID
        """
        assert isinstance(vector, kdl.Vector)
        assert isinstance(stamp, Time)
        assert isinstance(frame_id, str)
        self.vector = vector
        self.header = Header(frame_id=frame_id, stamp=stamp)

    def __repr__(self):
        xyz = f"(x={self.vector.x()}, y={self.vector.y()}, z={self.vector.z()})"
        return f"VectorStamped({xyz} @ {self.header.frame_id})"

    def __eq__(self, other):
        if isinstance(other, VectorStamped):
            return self.vector == other.vector and self.header.frame_id == other.header.frame_id
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash((self.vector, self.header.stamp, self.header.frame_id))

    @classmethod
    def from_xyz(cls, x: float, y: float, z: float, stamp: Time, frame_id: str) -> VectorStamped:
        """
        Custom constructor

        :param x: x
        :param y: y
        :param z: z
        :param stamp: TimeStamp
        :param frame_id: Frame ID
        :return: Filled object
        """
        vector = kdl.Vector(x, y, z)
        return cls(vector, stamp, frame_id)

    @classmethod
    def from_framestamped(cls, frame: FrameStamped) -> VectorStamped:
        """
        Custom constructor, extract vector from the frame

        :param frame: frame
        :return: Filled object
        """
        return cls(frame.frame.p, frame.header.stamp, frame.header.frame_id)
