from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from pytvc.data import rocket_motor


def clamp(x, min_val, max_val):
    return max(min(x, max_val), min_val)


@dataclass
class Vec3:

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
        """__init__ initializes a Vec3 object.

        Args:
            x (float, optional): the x component of the vector. Defaults to 0.0.
            y (float, optional): the y component of the vector. Defaults to 0.0.
            z (float, optional): the z component of the vector. Defaults to 0.0.
        """
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, other: Vec3) -> Vec3:
        """__add__ adds 2 vectors and returns the sum

        Args:
            other (Vec3): right hand vector to add

        Raises:
            NotImplementedError: raises notImplemented if the right hand side is not a vector object

        Returns:
            Vec3: a vector sum of the left hand and right hand vector
        """
        if isinstance(other, Vec3):
            return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)
        else:
            raise NotImplementedError

    def __sub__(self, other: Vec3) -> Vec3:
        """__sub__ subtracts 2 vectors and returns the difference

        Args:
            other (Vec3): right hand vector to subtract

        Raises:
            NotImplementedError: raises notImplemented if the right hand side is not a vector object

        Returns:
            Vec3: a vector difference of the left hand and right hand vector
        """
        if isinstance(other, Vec3):
            return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)
        else:
            raise NotImplementedError

    def __mul__(self, other: Vec3) -> Vec3:
        """__mul__ multiplies 2 vectors and returns the product

        Args:
            other (Vec3): right hand vector to multiply

        Raises:
            NotImplementedError: raises notImplemented if the right hand side is not a vector object

        Returns:
            Vec3: a vector product of the left hand and right hand vector
        """
        if isinstance(other, Vec3):
            return Vec3(self.x * other.x, self.y * other.y, self.z * other.z)
        elif isinstance(other, int) or isinstance(other, float):
            return Vec3(self.x * other, self.y * other, self.z * other)
        else:
            raise NotImplementedError

    def __truediv__(self, other) -> Vec3:
        """__truediv__ divides 2 vectors and returns the quotient

        Args:
            other: right hand vector to divide or scalar to divide by

        Raises:
            NotImplementedError: raises notImplemented if the right hand side is not a vector object or a number

        Returns:
            Vec3: a vector quotient of the left hand and right hand vector
        """
        if isinstance(other, Vec3):
            return Vec3(self.x / other.x, self.y / other.y, self.z / other.z)
        elif isinstance(other, int) or isinstance(other, float):
            return Vec3(self.x / other, self.y / other, self.z / other)
        else:
            raise NotImplementedError

    def __repr__(self) -> str:
        """__repr__ returns a string representation of the vector

        Returns:
            str: a string representation of the vector
        """
        return f"Vec3({self.x}, {self.y}, {self.z})"

    def __str__(self) -> str:
        """__str__ returns a string representation of the vector

        Returns:
            str: a string representation of the vector
        """
        return f"{self.x}, {self.y}, {self.z}"

    def __neg__(self) -> Vec3:
        """__neg__ returns the negative of the vector

        Returns:
            Vec3: the negative of the vector
        """
        return Vec3(-self.x, -self.y, -self.z)

    def __eq__(self, other: Vec3) -> bool:
        """__eq__ returns true if the vectors are equal

        Args:
            other (Vec3): right hand vector to compare

        Raises:
            NotImplementedError: raises notImplemented if the right hand side is not a vector object

        Returns:
            bool: true if the vectors are equal
        """
        if isinstance(other, Vec3):
            return self.x == other.x and self.y == other.y and self.z == other.z
        elif isinstance(other, int) or isinstance(other, float):
            return self.length() == other
        else:
            raise NotImplementedError

    def __ne__(self, other: Vec3) -> bool:
        """__ne__ returns true if the vectors are not equal

        Args:
            other (Vec3): right hand vector to compare

        Raises:
            NotImplementedError: raises notImplemented if the right hand side is not a vector object

        Returns:
            bool: true if the vectors are not equal
        """
        if isinstance(other, Vec3):
            return self.x != other.x or self.y != other.y or self.z != other.z
        else:
            print(other.__class__)
            raise NotImplementedError

    def __round__(self, ndigits: int = 0) -> Vec3:
        """__round__ returns the rounded vector

        Args:
            ndigits (int): number of digits to round to

        Returns:
            Vec3: the rounded vector
        """
        return Vec3(round(self.x, ndigits), round(self.y, ndigits), round(self.z, ndigits))

    def __abs__(self) -> Vec3:
        """__abs__ returns the absolute value of the vector

        Returns:
            Vec3: the absolute value of the vector
        """
        return Vec3(abs(self.x), abs(self.y), abs(self.z))

    def cross(self, other: Vec3) -> Vec3:
        """cross returns the cross product of 2 vectors

        Args:
            other (Vec3): right hand vector to cross

        Raises:
            NotImplementedError: raises notImplemented if the right hand side is not a vector object

        Returns:
            Vec3: a vector cross product of the left hand and right hand vector
        """
        if isinstance(other, Vec3):
            return Vec3(self.y * other.z - self.z * other.y, self.z * other.x - self.x * other.z, self.x * other.y - self.y * other.x)
        else:
            raise NotImplementedError

    def dot(self, other: Vec3) -> float:
        """dot returns the dot product of 2 vectors

        Args:
            other (Vec3): right hand vector to dot

        Raises:
            NotImplementedError: raises notImplemented if the right hand side is not a vector object

        Returns:
            float: a vector dot product of the left hand and right hand vector
        """
        if isinstance(other, Vec3):
            return self.x * other.x + self.y * other.y + self.z * other.z
        else:
            raise NotImplementedError

    def length(self) -> float:
        """length returns the length of the vector

        Returns:
            float: the length of the vector
        """
        return np.sqrt(self.x**2 + self.y**2 + self.z**2)

    def normalize(self) -> Vec3:
        """normalize returns the normalized vector

        Returns:
            Vec3: the normalized vector
        """
        return self / self.length()

    def angle_between_vectors(self, vector: Vec3) -> float:
        """Calculate the angle between two vectors."""
        if isinstance(vector, Vec3):
            return np.arccos(self.dot(vector) / (self.length() * vector.length()))
        else:
            return None


@dataclass
class Quat:

    w: float = 1.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __init__(self, w: float = 1.0, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        """__init__ initializes a Quat object

        Args:
            w (float, optional): the real component of the Quat. Defaults to 1.0.
            x (float, optional): the x component of the Quat. Defaults to 0.0.
            y (float, optional): the y component of the Quat. Defaults to 0.0.
            z (float, optional): the z component of the Quat. Defaults to 0.0.
        """
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def __mul__(self, other: Quat) -> Quat:
        """__mul__ multiplies 2 Quats and returns the product

        Args:
            other (Quat): right hand Quat to multiply

        Raises:
            NotImplementedError: raises notImplemented if the right hand side is not a Quat object

        Returns:
            Quat: a Quat product of the left hand and right hand Quat
        """
        if isinstance(other, Quat):
            return Quat(
                self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
                self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
                self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
                self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w
            )
        else:
            raise NotImplementedError

    def __truediv__(self, scalar: float) -> Quat:
        """__truediv__ _summary_

        Args:
            scalar (float): _description_

        Returns:
            Quat: _description_
        """

        if isinstance(scalar, float) or isinstance(scalar, int):
            return Quat(self.w / scalar, self.x / scalar, self.y / scalar, self.z / scalar)

    def __add__(self, other: Quat) -> Quat:
        """__add__ adds 2 Quats and returns the sum

        Args:
            other (Quat): right hand Quat to add

        Raises:
            NotImplementedError: raises notImplemented if the right hand side is not a Quat object

        Returns:
            Quat: a Quat sum of the left hand and right hand Quat
        """
        if isinstance(other, Quat):
            return Quat(self.w + other.w, self.x + other.x, self.y + other.y, self.z + other.z)
        else:
            raise NotImplementedError

    def __sub__(self, other: Quat) -> Quat:
        """__sub__ subtracts 2 Quats and returns the difference

        Args:
            other (Quat): right hand Quat to subtract

        Raises:
            NotImplementedError: raises notImplemented if the right hand side is not a Quat object

        Returns:
            Quat: a Quat difference of the left hand and right hand Quat
        """
        if isinstance(other, Quat):
            return Quat(self.w - other.w, self.x - other.x, self.y - other.y, self.z - other.z)
        else:
            raise NotImplementedError

    def __repr__(self) -> str:
        """__repr__ returns the Quat as a string

        Returns:
            str: the Quat as a string
        """
        return f"Quat({self.w}, {self.x}, {self.y}, {self.z})"

    def __str__(self) -> str:
        """__str__ returns the Quat as a string

        Returns:
            str: the Quat as a string
        """
        return f"Quat({self.w}, {self.x}, {self.y}, {self.z})"

    def length(self) -> float:
        """length returns the length of the Quat

        Returns:
            float: the length of the Quat
        """
        return np.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)

    def normalize(self) -> Quat:
        """normalize returns the normalized Quat

        Returns:
            Quat: the normalized Quat
        """
        return self / self.length()

    def conjugate(self) -> Quat:
        """conjugate returns the conjugate of the Quat

        Returns:
            Quat: the conjugate of the Quat
        """
        return Quat(self.w, -self.x, -self.y, -self.z)

    def rotate(self, vector: Vec3) -> Vec3:
        """rotate returns the rotated vector

        Args:
            vector (Vec3): vector to rotate

        Raises:
            NotImplementedError: raises notImplemented if the right hand side is not a vector object

        Returns:
            Vec3: the rotated vector
        """
        if isinstance(vector, Vec3):
            tmp: Quat = Quat(0, vector.x, vector.y, vector.z)
            tmp = self * tmp * self.conjugate()
            return Vec3(tmp.x, tmp.y, tmp.z)
        else:
            raise NotImplementedError(f"cannot rotate type {vector.__class__}")

    def fractional(self, a) -> Quat:
        """Return the fractional of the Quat."""

        self.w = 1-a + a*self.w
        self.x *= a
        self.y *= a
        self.z *= a

        return self.norm()

    def rotation_between_vectors(self, vector: Vec3) -> Quat:
        """Return the rotation Quat between two vectors.

        parameters:

        vector : vector3
            vector to rotate
        """

        q = Quat(0.0, vector.x, vector.y, vector.z)

        q = self * q
        q.w = 1 - q.w

        return q.normalize()

    def from_axis_angle(self, axis: Vec3, angle: float) -> Quat:
        """Return the Quat from an axis and angle.

        parameters:

        axis : vector3
            axis of rotation

        angle : float
            angle of rotation
        """

        sa: float = np.sin(angle / 2)

        self.w = np.cos(angle / 2)
        self.x = axis.x * sa
        self.y = axis.y * sa
        self.z = axis.z * sa

        return self

    def from_euler(self, euler_angles: Vec3) -> Quat:
        """set the Quat from euler angles

        Args:
            euler_angles (Vec3): euler angles to set to

        Raises:
            NotImplementedError: raises notImplemented if the right hand side is not a vector object

        Returns:
            Quat: the Quat set from euler angles
        """

        cr = np.cos(euler_angles.x / 2.0)
        cp = np.cos(euler_angles.y / 2.0)
        cy = np.cos(euler_angles.z / 2.0)

        sr = np.sin(euler_angles.x / 2.0)
        sp = np.sin(euler_angles.y / 2.0)
        sy = np.sin(euler_angles.z / 2.0)

        self.w = cr * cp * cy + sr * sp * sy
        self.x = sr * cp * cy - cr * sp * sy
        self.y = cr * sp * cy + sr * cp * sy
        self.z = cr * cp * sy - sr * sp * cy

        return self

    def to_euler(self) -> Vec3:
        """Convert a Quat to euler angles."""
        x = np.arctan2(2.0 * (self.w * self.x + self.y * self.z),
                       1.0 - 2.0 * (self.x**2 + self.y**2))
        y = np.arcsin(2.0 * (self.w * self.y - self.z * self.x))
        z = np.arctan2(2.0 * (self.w * self.z + self.x * self.y),
                       1.0 - 2.0 * (self.y**2 + self.z**2))

        return Vec3(x, y, z)

class physics_body:

    """physics_body"""

    def __init__(self, position: Vec3 = Vec3(), velocity: Vec3 = Vec3(), rotation: Quat = Quat(), rotational_velocity: Vec3 = Vec3(),
                 mass: float = 1.0, moment_of_inertia: Vec3 = Vec3(1.0, 1.0, 1.0), ref_area: float = 1.0, drag_coefficient_forewards: float = 0.0,
                 drag_coefficient_sideways: float = 0.0, wind_speed: Vec3 = Vec3(), cp_location: Vec3 = Vec3(), friction_coeff: float = 0.0, use_aero: bool = False):
        """initializes the physics_body object

        Args:
            position (Vec3, optional): position of the body. Defaults to Vec3.
            velocity (Vec3, optional): velocity of the body. Defaults to Vec3.
            rotation (Quat, optional): rotation of the body. Defaults to Quat.
            rotational_velocity (Vec3, optional): rotational velocity of the body. Defaults to Vec3.
            mass (float, optional): mass of the body. Defaults to 1.0.
            moment_of_inertia (Vec3, optional): inertia of the body. Defaults to Vec3.
            ref_area (float, optional): reference area of the body. Defaults to 1.0.
            drag_coefficient_forewards (float, optional): drag coefficient of the body in the forewards direction. Defaults to 0.0.
            drag_coefficient_sideways (float, optional): drag coefficient of the body in the sideways direction. Defaults to 0.0.
            wind_speed (Vec3, optional): speed of wind relative to the body. Defaults to Vec3.
            cp_location (Vec3, optional): center of pressure location of the body. Defaults to Vec3.
            friction_coeff (float, optional): friction coefficient of the body. Defaults to 0.0.
            use_aero (bool, optional): whether or not to use aerodynamic forces. Defaults to False.
        """

        self.position: Vec3 = position
        self.velocity: Vec3 = velocity

        self.acceleration: Vec3 = Vec3(0.0, 0.0, 0.0)
        self.acceleration_local: Vec3 = Vec3(0.0, 0.0, 0.0)

        self.rotation: Quat = rotation
        self.rotation_euler: Vec3 = Vec3(0.0, 0.0, 0.0)

        self.rotational_velocity: Vec3 = rotational_velocity
        self.rotational_acceleration: Vec3 = Vec3(0.0, 0.0, 0.0)

        self.mass: float = mass
        self.moment_of_inertia: Vec3 = moment_of_inertia

        self.aoa: float = 0.0
        self.ref_area: float = ref_area

        self.drag_coefficient_forwards: float = drag_coefficient_forewards
        self.drag_coefficient_sideways: float = drag_coefficient_sideways

        self.wind_speed: Vec3 = wind_speed

        self.cp_location: Vec3 = cp_location

        self.friction_coeff: float = friction_coeff
        self.air_density: float = 1.225

        self.drag_force: Vec3 = Vec3()

        self.use_aero: bool = use_aero

    def apply_torque(self, torque: Vec3) -> None:
        """apply a torque on the body in the global frame

        Args:
            torque (Vec3): _description_
        """
        self.rotational_acceleration += (torque / self.moment_of_inertia)

    def apply_local_torque(self, torque: Vec3) -> None:
        """apply a torque in the local frame

        Args:
            torque (Vec3): the torque to apply
        """
        self.apply_torque(self.rotation.rotate(torque))

    def apply_point_torque(self, force: Vec3, point: Vec3) -> None:
        """apply a point torque in the global frame

        Args:
            torque (Vec3): the force to apply
            point (Vec3): distance of the force from the center of mass
        """
        tmp = point.cross(force)
        self.apply_torque(tmp)

    def apply_local_point_torque(self, force: Vec3, point: Vec3) -> None:
        """apply a point torque in the local frame

        Args:
            force (Vec3): the force to apply 
            point (Vec3): the distance of the force from the center of mass
        """
        self.apply_point_torque(self.rotation.rotate(force), self.rotation.rotate(point))

    def apply_force(self, force: Vec3) -> None:
        """apply a force on the body in the global frame

        Args:
            force (Vec3): the force to apply
        """
        accel: Vec3 = force / self.mass
        self.acceleration += accel
    
    def apply_local_force(self, force: Vec3) -> None:
        """apply a force in the local frame

        Args:
            force (Vec3): the force to apply
        """
        self.apply_force(self.rotation.rotate(force))

    def apply_point_force(self, force: Vec3, point: Vec3) -> None:
        """apply a point force in the global frame, affects rotation and translation

        Args:
            force (Vec3): the force to apply
            point (Vec3): the distance of the force from the center of mass
        """
        self.apply_force(force)
        self.apply_point_torque(force, point)

    def apply_local_point_force(self, force: Vec3, point: Vec3) -> None:
        """apply a point force in the local frame, affects rotation and translation

        Args:
            force (Vec3): the force to apply
            point (Vec3): the distance of the force from the bodies center of mass
        """
        self.apply_point_force(self.rotation.rotate(force), self.rotation.rotate(point))

    def update(self, dt: float) -> None:
        """updates the physics body for the given change in time

        Args:
            dt (float): change in time from the previous update to "now"
        """

        # aero
        velocity_relative_wind: Vec3 = self.velocity - self.wind_speed
        if velocity_relative_wind.length() > 0.0:

            self.aoa = velocity_relative_wind.angle_between_vectors(
                self.rotation.rotate(Vec3(1.0, 0.0, 0.0)))

            if self.aoa <= 1e-5:
                self.aoa = 1e-5

            if (self.aoa > 1.5708):
                self.aoa = np.pi - self.aoa

            drag_coefficient = self.drag_coefficient_forwards + \
                ((-np.cos(self.aoa)/2.0 + 0.5) *
                 (self.drag_coefficient_sideways - self.drag_coefficient_forwards))

            self.drag_force = -velocity_relative_wind.normalize() * (drag_coefficient/2.0 *
                                                                     self.air_density * self.ref_area * (self.velocity.length()*self.velocity.length()))

            self.apply_point_force(self.drag_force, self.cp_location)
            self.apply_torque(self.rotational_velocity * -
                              self.friction_coeff * self.air_density)

        self.acceleration_local = self.rotation.conjugate().rotate(self.acceleration)

        self.acceleration.x -= 9.81

        self.position += self.velocity * dt + (self.acceleration*0.5) * (dt*dt)
        self.velocity += self.acceleration * dt

        self.rotational_velocity += self.rotational_acceleration * dt

        ang: float = self.rotational_velocity.length()
        if ang > 0.0:
            self.rotation *= Quat().from_axis_angle(self.rotational_velocity / ang, ang * dt)

        self.rotation_euler = self.rotation.to_euler()

        if self.position.x < 0.0:
            self.position.x = 0.0
            self.velocity = Vec3()
            # self.rotational_velocity = Vec3()

    def clear(self):
        """clears the physics body of all forces and torques"""
        self.acceleration = Vec3(0.0, 0.0, 0.0)
        self.rotational_acceleration = Vec3(0.0, 0.0, 0.0)
