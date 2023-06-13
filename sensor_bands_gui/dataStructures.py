from enum import Enum
from typing import Generator

import attr


class Constants(Enum):
    """
    Got directly from main.cpp sent by Muhammad
    """

    NUM_FSR = 8
    NUM_FSR_FRAMES = 1
    NUM_FRAMES = 1
    NUM_IMU = 13  # 3xGyro, 3xAcc, 4xQuaternion, 3xGravity


@attr.define
class Vector3:
    x: float = attr.field(validator=attr.validators.instance_of((int, float)))
    y: float = attr.field(validator=attr.validators.instance_of((int, float)))
    z: float = attr.field(validator=attr.validators.instance_of((int, float)))

    def __add__(self, v: "Vector3") -> "Vector3":
        return Vector3(self.x + v.x, self.y + v.y, self.z + v.z)

    def __floordiv__(self, scalar: float) -> "Vector3":
        return Vector3(self.x // scalar, self.y // scalar, self.z // scalar)

    def __truediv__(self, scalar: float) -> "Vector3":
        return Vector3(self.x / scalar, self.y / scalar, self.z / scalar)

    def __rmul__(self, scalar: float) -> "Vector3":
        return Vector3(self.x * scalar, self.y * scalar, self.z * scalar)

    def __mul__(self, scalar: float) -> "Vector3":
        return scalar * self

    def __neg__(self) -> "Vector3":
        return Vector3(-self.x, -self.y, -self.z)

    @staticmethod
    def dot(v1: "Vector3", v2: "Vector3") -> float:
        return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z

    @staticmethod
    def cross(v1: "Vector3", v2: "Vector3") -> "Vector3":
        s1 = v1.y * v2.z - v1.z * v2.y
        s2 = v1.z * v2.x - v1.x * v2.z
        s3 = v1.x * v2.y - v1.y * v2.x
        return Vector3(s1, s2, s3)

    @staticmethod
    def sum(
        it: tuple["Vector3"] | list["Vector3"] | Generator["Vector3", None, None]
    ) -> "Vector3":
        out = Vector3(0, 0, 0)
        for vector in it:
            out += vector
        return out

    def magnitude(self) -> float:
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def normalize(self) -> None:
        norm = self.magnitude()
        self.x /= norm
        self.y /= norm
        self.z /= norm

    def to_list(self) -> list[float]:
        return [self.x, self.y, self.z]


class Axis(Enum):
    X = 0
    Y = 1
    Z = 2


class Quaternion:
    v: Vector3
    w: float

    def __init__(self, *args) -> None:
        """
        Quaterion(x, y, z, w)
        """
        error_message = "The arguments must be 4 numbers or a Vectro3 and a number"
        if len(args) == 1 and type(args[0]) in [tuple, list]:
            args = args[0]
        if any(type(el) not in [float, int, Vector3] for el in args):
            raise TypeError(error_message)
        if len(args) == 4:
            self.v = Vector3(*args[:3])
            self.w = args[3]
        elif len(args) == 2:
            if isinstance(args[0], Vector3):
                self.v = args[0]
                self.w = args[1]
            elif isinstance(args[1], Vector3):
                self.v = args[1]
                self.w = args[0]
            else:
                raise TypeError(error_message)
        else:
            raise TypeError(error_message)

    def __repr__(self) -> str:
        return f"v: {self.v}, w: {self.w}"

    def __mul__(self, q: "Quaternion") -> "Quaternion":
        w = self.w * q.w - Vector3.dot(self.v, q.v)
        v = self.w * q.v + q.w * self.v + Vector3.cross(self.v, q.v)
        return Quaternion(v, w)

    def __eq__(self, q: "Quaternion") -> bool:
        return self.w == q.w and self.v == q.v

    def conjugate(self) -> "Quaternion":
        return Quaternion(self.w, -self.v)

    def magnitude(self) -> float:
        return math.sqrt(self.w * self.w + self.v.magnitude() ** 2)

    def normalize(self) -> None:
        if mag := self.magnitude():
            self.w = self.w / mag
            self.v = self.v / mag

    def to_euler(self, *, degrees: bool = True, correct: bool = True) -> Vector3:
        """
        Rotation order = ZYX
        """
        if correct:
            self.apply_rotation_single_axis(90, Axis.Y)
        pitch = math.asin(self._correct_asin_float_imprecision())
        if pitch == math.pi / 2:
            roll = 0
            yaw = -2 * math.atan2(self.v.x, self.w)
        elif pitch == -math.pi / 2:
            roll = 0
            yaw = 2 * math.atan2(self.v.x, self.w)
        else:
            roll = math.atan2(
                self.w * self.v.x + self.v.y * self.v.z,
                1 / 2 - (self.v.x**2 + self.v.y**2),
            )
            yaw = math.atan2(
                self.w * self.v.z + self.v.x * self.v.y,
                1 / 2 - (self.v.y**2 + self.v.z**2),
            )

        pitch *= -1

        if degrees:
            pitch = math.degrees(pitch)
            roll = math.degrees(roll)
            yaw = math.degrees(yaw)

        return Vector3(roll, pitch, yaw)

    def _correct_asin_float_imprecision(self) -> float:
        self.normalize()
        inner = 2 * (self.w * self.v.y - self.v.x * self.v.z)
        return 1 if inner > 1 else max(inner, -1)

    def apply_rotation(self, q: "Quaternion") -> None:
        new = self * q
        self.w = new.w
        self.v = new.v

    def apply_rotation_single_axis(
        self, angle_degrees: float | int, axis: Axis
    ) -> None:
        sin = math.sin(math.radians(angle_degrees) / 2)
        vector_components = [0.0, 0.0, 0.0]
        vector_components[axis.value] = sin
        v = Vector3(*vector_components)
        w = math.cos(math.radians(angle_degrees) / 2)
        self.apply_rotation(Quaternion(v, w))
