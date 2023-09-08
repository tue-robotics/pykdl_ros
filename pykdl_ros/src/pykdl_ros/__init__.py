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
        pos = f"(x={self.frame.p.x()}, y={self.frame.p.y()}, z={self.frame.p.z()})"
        r, p, y = self.frame.M.GetRPY()
        rot = f"(r={r}, p={p}, y={y})"
        return f"FrameStamped({pos=}, {rot=} @ {self.header.frame_id})"

    def __eq__(self, other):
        if isinstance(other, FrameStamped):
            return self.frame == other.frame and self.header.frame_id == other.header.frame_id
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash((self.frame, self.header.frame_id))

    @classmethod
    def identity(cls, stamp: Time, frame_id: str) -> FrameStamped:
        """
        Construct a FrameStamped object with identity frame.

        :param stamp: TimeStamp
        :param frame_id: Frame ID
        :return: Filled object
        """
        frame = kdl.Frame.Identity()
        return cls(frame, stamp, frame_id)

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


class TwistStamped:
    """Stamped version of PyKDL.Twist."""

    __slots__ = "twist", "header"

    def __init__(self, twist: kdl.Twist, stamp: Time, frame_id: str):
        """
        Construct a TwistStamped object.

        :param twist: twist
        :param stamp: TimeStamp
        :param frame_id: Frame ID
        """
        assert isinstance(twist, kdl.Twist)
        assert isinstance(stamp, Time)
        assert isinstance(frame_id, str)
        self.twist = twist
        self.header = Header(frame_id=frame_id, stamp=stamp)

    def __repr__(self):
        vel = f"(x={self.twist.vel.x()}, y={self.twist.vel.y()}, z={self.twist.vel.z()})"
        rot = f"(r={self.twist.rot.x()}, p={self.twist.rot.y()}, y={self.twist.rot.z()})"
        return f"TwistStamped({vel=}, {rot=} @ {self.header.frame_id})"

    def __eq__(self, other):
        if isinstance(other, TwistStamped):
            return self.twist == other.twist and self.header.frame_id == other.header.frame_id
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash((self.twist, self.header.frame_id))

    @classmethod
    def zero(cls, stamp: Time, frame_id: str) -> TwistStamped:
        """
        Construct a TwistStamped object with zero velocity and angular velocity.

        :param stamp: TimeStamp
        :param frame_id: Frame ID
        :return: Filled object
        """
        twist = kdl.Twist.Zero()
        return cls(twist, stamp, frame_id)

    @classmethod
    def from_xyz_rpy(cls, vx: float, vy: float, vz: float, wx: float, wy: float, wz: float, stamp: Time, frame_id: str):
        """
        Construct a TwistStamped from velocity and XYZ and RPY.

        :param vx: vx
        :param vy: vy
        :param vz: vz
        :param wx: wx
        :param wy: wy
        :param wz: wz
        :param stamp: TimeStamp
        :param frame_id: Frame ID
        :return: Filled object
        """
        linear = kdl.Vector(vx, vy, vz)
        angular = kdl.Vector(wx, wy, wz)
        twist = kdl.Twist(linear, angular)
        return cls(twist, stamp, frame_id)


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
        return hash((self.vector, self.header.frame_id))

    @classmethod
    def zero(cls, stamp: Time, frame_id: str) -> VectorStamped:
        """
        Construct a VectorStamped object with zero vector.

        :param stamp: TimeStamp
        :param frame_id: Frame ID
        :return: Filled object
        """
        vector = kdl.Vector.Zero()
        return cls(vector, stamp, frame_id)

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


class WrenchStamped:
    """Stamped version of PyKDL.Wrench."""

    __slots__ = "wrench", "header"

    def __init__(self, wrench: kdl.Wrench, stamp: Time, frame_id: str):
        """
        Construct a WrenchStamped object.

        :param wrench: wrench
        :param stamp: TimeStamp
        :param frame_id: Frame ID
        """
        assert isinstance(wrench, kdl.Wrench)
        assert isinstance(stamp, Time)
        assert isinstance(frame_id, str)
        self.wrench = wrench
        self.header = Header(frame_id=frame_id, stamp=stamp)

    def __repr__(self):
        force = f"(x={self.wrench.force.x()}, y={self.wrench.force.y()}, z={self.wrench.force.z()})"
        torque = f"(x={self.wrench.torque.x()}, y={self.wrench.torque.y()}, z={self.wrench.torque.z()})"
        return f"WrenchStamped({force=}, {torque=} @ {self.header.frame_id})"

    def __eq__(self, other):
        if isinstance(other, WrenchStamped):
            return self.wrench == other.wrench and self.header.frame_id == other.header.frame_id
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash((self.wrench, self.header.frame_id))

    @classmethod
    def zero(cls, stamp: Time, frame_id: str) -> WrenchStamped:
        """
        Construct a WrenchStamped object with zero force and torque.

        :param stamp: TimeStamp
        :param frame_id: Frame ID
        :return: Filled object
        """
        wrench = kdl.Wrench.Zero()
        return cls(wrench, stamp, frame_id)

    @classmethod
    def from_fxfyfz_txtytz(
        cls, fx: float, fy: float, fz: float, tx: float, ty: float, tz: float, stamp: Time, frame_id: str
    ):
        """
        Construct a WrenchStamped from force and torque in XYZ.

        :param fx: fx
        :param fy: fy
        :param fz: fz
        :param tx: tx
        :param ty: ty
        :param tz: tz
        :param stamp: TimeStamp
        :param frame_id: Frame ID
        :return: Filled object
        """
        force = kdl.Vector(fx, fy, fz)
        torque = kdl.Vector(tx, ty, tz)
        wrench = kdl.Wrench(force, torque)
        return cls(wrench, stamp, frame_id)
