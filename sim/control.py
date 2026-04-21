"""PID controllers for PyBullet drone simulation.

Ported from ``gym-pybullet-drones-routing/gym_pybullet_drones/control/``.

Classes
-------
DSLPIDControl
    Position PID for Crazyflie drones (CF2X, CF2P, RACE).
PIDVelocityControl
    Velocity PID for HB (Hummingbird) drone.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as etxml
from pathlib import Path

import numpy as np
import pybullet as p
from scipy.optimize import nnls
from scipy.spatial.transform import Rotation

from sim.enums import DroneModel

ASSETS_DIR = Path(__file__).resolve().parent / "assets"


# ======================================================================
# Utility
# ======================================================================

def nnlsRPM(
    thrust: float, x_torque: float, y_torque: float, z_torque: float,
    counter: int, max_thrust: float, max_xy_torque: float,
    max_z_torque: float, a: np.ndarray, inv_a: np.ndarray,
    b_coeff: np.ndarray, gui: bool = False,
) -> np.ndarray:
    """Non-negative Least Squares RPMs from desired thrust and torques."""
    B = np.multiply(np.array([thrust, x_torque, y_torque, z_torque]), b_coeff)
    sq_rpm = np.dot(inv_a, B)
    if np.min(sq_rpm) < 0:
        sol, _ = nnls(a, B, maxiter=3 * a.shape[1])
        sq_rpm = sol
    return np.sqrt(np.clip(sq_rpm, 0, None))


def _getURDFParameter(drone_model: DroneModel, parameter_name: str) -> float:
    """Read a single scalar from the drone's URDF file."""
    urdf = drone_model.value + ".urdf"
    path = ASSETS_DIR / urdf
    tree = etxml.parse(str(path)).getroot()
    if parameter_name == "m":
        return float(tree[1][0][1].attrib["value"])
    elif parameter_name in ("ixx", "iyy", "izz"):
        return float(tree[1][0][2].attrib[parameter_name])
    elif parameter_name in (
        "arm", "thrust2weight", "kf", "km", "max_speed_kmh",
        "gnd_eff_coeff", "prop_radius",
        "drag_coeff_xy", "drag_coeff_z",
        "dw_coeff_1", "dw_coeff_2", "dw_coeff_3",
    ):
        return float(tree[0].attrib[parameter_name])
    elif parameter_name in ("length", "radius"):
        return float(tree[1][2][1][0].attrib[parameter_name])
    elif parameter_name == "collision_z_offset":
        offsets = [float(s) for s in tree[1][2][0].attrib["xyz"].split(" ")]
        return offsets[2]
    raise ValueError(f"Unknown URDF parameter: {parameter_name}")


# ======================================================================
# DSLPIDControl — position PID (Crazyflie / Racer)
# ======================================================================

class DSLPIDControl:
    """PID position controller for Crazyflie-class drones.

    Based on work from UTIAS Dynamic Systems Lab.
    """

    def __init__(self, drone_model: DroneModel, g: float = 9.8):
        self.DRONE_MODEL = drone_model
        self.GRAVITY = g * _getURDFParameter(drone_model, "m")
        self.KF = _getURDFParameter(drone_model, "kf")
        self.KM = _getURDFParameter(drone_model, "km")

        self.P_COEFF_FOR = np.array([0.4, 0.4, 1.25])
        self.I_COEFF_FOR = np.array([0.05, 0.05, 0.05])
        self.D_COEFF_FOR = np.array([0.2, 0.2, 0.5])
        self.P_COEFF_TOR = np.array([70000.0, 70000.0, 60000.0])
        self.I_COEFF_TOR = np.array([0.0, 0.0, 500.0])
        self.D_COEFF_TOR = np.array([20000.0, 20000.0, 12000.0])
        self.PWM2RPM_SCALE = 0.2685
        self.PWM2RPM_CONST = 4070.3
        self.MIN_PWM = 20000
        self.MAX_PWM = 65535

        if self.DRONE_MODEL in (DroneModel.CF2X, DroneModel.RACE):
            self.MIXER_MATRIX = np.array([
                [-0.5, -0.5, -1],
                [-0.5,  0.5,  1],
                [ 0.5,  0.5, -1],
                [ 0.5, -0.5,  1],
            ])
        elif self.DRONE_MODEL == DroneModel.CF2P:
            self.MIXER_MATRIX = np.array([
                [ 0, -1, -1],
                [ 1,  0,  1],
                [ 0,  1, -1],
                [-1,  0,  1],
            ])
        else:
            self.MIXER_MATRIX = np.array([
                [-0.5, -0.5, -1],
                [-0.5,  0.5,  1],
                [ 0.5,  0.5, -1],
                [ 0.5, -0.5,  1],
            ])
        self.reset()

    def reset(self):
        self.control_counter = 0
        self.last_rpy = np.zeros(3)
        self.last_pos_e = np.zeros(3)
        self.integral_pos_e = np.zeros(3)
        self.last_rpy_e = np.zeros(3)
        self.integral_rpy_e = np.zeros(3)

    def computeControl(
        self,
        control_timestep: float,
        cur_pos: np.ndarray,
        cur_quat: np.ndarray,
        cur_vel: np.ndarray,
        cur_ang_vel: np.ndarray,
        target_pos: np.ndarray,
        target_rpy: np.ndarray = np.zeros(3),
        target_vel: np.ndarray = np.zeros(3),
        target_rpy_rates: np.ndarray = np.zeros(3),
    ) -> tuple[np.ndarray, np.ndarray, float]:
        """Compute RPMs to track ``target_pos``.

        Returns ``(rpm, pos_error, yaw_error)``.
        """
        self.control_counter += 1
        thrust, computed_target_rpy, pos_e = self._positionPID(
            control_timestep, cur_pos, cur_quat, cur_vel,
            target_pos, target_rpy, target_vel,
        )
        rpm = self._attitudePID(
            control_timestep, thrust, cur_quat,
            computed_target_rpy, target_rpy_rates,
        )
        cur_rpy = p.getEulerFromQuaternion(cur_quat)
        return rpm, pos_e, computed_target_rpy[2] - cur_rpy[2]

    # ---- internal PID loops -----------------------------------------

    def _positionPID(self, dt, cur_pos, cur_quat, cur_vel,
                     target_pos, target_rpy, target_vel):
        cur_rotation = np.array(p.getMatrixFromQuaternion(cur_quat)).reshape(3, 3)
        pos_e = target_pos - cur_pos
        vel_e = target_vel - cur_vel
        self.integral_pos_e = self.integral_pos_e + pos_e * dt
        self.integral_pos_e = np.clip(self.integral_pos_e, -2.0, 2.0)
        self.integral_pos_e[2] = np.clip(self.integral_pos_e[2], -0.15, 0.15)

        target_thrust = (
            np.multiply(self.P_COEFF_FOR, pos_e)
            + np.multiply(self.I_COEFF_FOR, self.integral_pos_e)
            + np.multiply(self.D_COEFF_FOR, vel_e)
            + np.array([0, 0, self.GRAVITY])
        )
        scalar_thrust = max(0.0, np.dot(target_thrust, cur_rotation[:, 2]))
        thrust = (math.sqrt(scalar_thrust / (4 * self.KF)) - self.PWM2RPM_CONST) / self.PWM2RPM_SCALE

        target_z_ax = target_thrust / np.linalg.norm(target_thrust)
        target_x_c = np.array([math.cos(target_rpy[2]), math.sin(target_rpy[2]), 0])
        target_y_ax = np.cross(target_z_ax, target_x_c)
        target_y_ax /= np.linalg.norm(target_y_ax)
        target_x_ax = np.cross(target_y_ax, target_z_ax)
        target_rotation = np.vstack([target_x_ax, target_y_ax, target_z_ax]).T
        target_euler = Rotation.from_matrix(target_rotation).as_euler("XYZ", degrees=False)
        return thrust, target_euler, pos_e

    def _attitudePID(self, dt, thrust, cur_quat, target_euler, target_rpy_rates):
        cur_rotation = np.array(p.getMatrixFromQuaternion(cur_quat)).reshape(3, 3)
        cur_rpy = np.array(p.getEulerFromQuaternion(cur_quat))
        target_quat = Rotation.from_euler("XYZ", target_euler, degrees=False).as_quat()
        w, x, y, z = target_quat
        target_rotation = Rotation.from_quat([w, x, y, z]).as_matrix()
        rot_e_mat = np.dot(target_rotation.T, cur_rotation) - np.dot(cur_rotation.T, target_rotation)
        rot_e = np.array([rot_e_mat[2, 1], rot_e_mat[0, 2], rot_e_mat[1, 0]])
        rpy_rates_e = target_rpy_rates - (cur_rpy - self.last_rpy) / dt
        self.last_rpy = cur_rpy
        self.integral_rpy_e = self.integral_rpy_e - rot_e * dt
        self.integral_rpy_e = np.clip(self.integral_rpy_e, -1500.0, 1500.0)
        self.integral_rpy_e[0:2] = np.clip(self.integral_rpy_e[0:2], -1.0, 1.0)

        target_torques = (
            -np.multiply(self.P_COEFF_TOR, rot_e)
            + np.multiply(self.D_COEFF_TOR, rpy_rates_e)
            + np.multiply(self.I_COEFF_TOR, self.integral_rpy_e)
        )
        target_torques = np.clip(target_torques, -3200, 3200)
        pwm = thrust + np.dot(self.MIXER_MATRIX, target_torques)
        pwm = np.clip(pwm, self.MIN_PWM, self.MAX_PWM)
        return self.PWM2RPM_SCALE * pwm + self.PWM2RPM_CONST


# ======================================================================
# PIDVelocityControl — velocity PID (Hummingbird)
# ======================================================================

class PIDVelocityControl:
    """Velocity-tracking PID controller for DroneModel.HB."""

    def __init__(self, drone_model: DroneModel, g: float = 9.8):
        self.DRONE_MODEL = drone_model
        self.GRAVITY = g * _getURDFParameter(drone_model, "m")
        self.KF = _getURDFParameter(drone_model, "kf")
        self.KM = _getURDFParameter(drone_model, "km")

        self.P_COEFF_VEL = np.array([0.3, 0.3, 0.5])
        self.I_COEFF_VEL = np.array([0.05, 0.05, 0.1])
        self.D_COEFF_VEL = np.array([0.02, 0.02, 0.03])
        self.P_COEFF_TOR = np.array([0.18, 0.18, 0.18])
        self.I_COEFF_TOR = np.array([0.018, 0.018, 0.018])
        self.D_COEFF_TOR = np.array([0.002, 0.002, 0.002])

        self.MAX_ROLL_PITCH = np.pi / 6
        self.L = _getURDFParameter(drone_model, "arm")
        t2w = _getURDFParameter(drone_model, "thrust2weight")
        self.MAX_RPM = np.sqrt((t2w * self.GRAVITY) / (4 * self.KF))
        self.MAX_THRUST = 4 * self.KF * self.MAX_RPM ** 2
        self.MAX_XY_TORQUE = self.L * self.KF * self.MAX_RPM ** 2
        self.MAX_Z_TORQUE = 2 * self.KM * self.MAX_RPM ** 2

        self.A = np.array([
            [1,  1,  1,  1],
            [0,  1,  0, -1],
            [-1, 0,  1,  0],
            [-1, 1, -1,  1],
        ])
        self.INV_A = np.linalg.inv(self.A)
        self.B_COEFF = np.array([
            1 / self.KF, 1 / (self.KF * self.L),
            1 / (self.KF * self.L), 1 / self.KM,
        ])
        self.reset()

    def reset(self):
        self.control_counter = 0
        self.last_vel_e = np.zeros(3)
        self.integral_vel_e = np.zeros(3)
        self.last_rpy_e = np.zeros(3)
        self.integral_rpy_e = np.zeros(3)

    def computeControl(
        self,
        control_timestep: float,
        cur_pos: np.ndarray,
        cur_quat: np.ndarray,
        cur_vel: np.ndarray,
        cur_ang_vel: np.ndarray,
        target_vel: np.ndarray,
        target_rpy: np.ndarray = np.zeros(3),
    ) -> tuple[np.ndarray, np.ndarray, float]:
        """Compute RPMs to track ``target_vel``.

        Returns ``(rpm, vel_error, yaw_error)``.
        """
        self.control_counter += 1
        thrust, computed_rpy, vel_e = self._velocityPID(
            control_timestep, cur_vel, cur_quat, target_vel,
        )
        rpm = self._attitudePID(control_timestep, thrust, cur_quat, computed_rpy)
        cur_rpy = p.getEulerFromQuaternion(cur_quat)
        return rpm, vel_e, computed_rpy[2] - cur_rpy[2]

    def _velocityPID(self, dt, cur_vel, cur_quat, target_vel):
        vel_e = target_vel - np.array(cur_vel).reshape(3)
        d_vel_e = (vel_e - self.last_vel_e) / dt
        self.last_vel_e = vel_e
        self.integral_vel_e += vel_e * dt

        target_force = (
            np.array([0, 0, self.GRAVITY])
            + np.multiply(self.P_COEFF_VEL, vel_e)
            + np.multiply(self.I_COEFF_VEL, self.integral_vel_e)
            + np.multiply(self.D_COEFF_VEL, d_vel_e)
        )
        target_rpy = np.zeros(3)
        sign_z = np.sign(target_force[2]) or 1
        target_rpy[0] = np.arcsin(-sign_z * target_force[1] / np.linalg.norm(target_force))
        target_rpy[1] = np.arctan2(sign_z * target_force[0], sign_z * target_force[2])
        target_rpy[0] = np.clip(target_rpy[0], -self.MAX_ROLL_PITCH, self.MAX_ROLL_PITCH)
        target_rpy[1] = np.clip(target_rpy[1], -self.MAX_ROLL_PITCH, self.MAX_ROLL_PITCH)

        cur_rotation = np.array(p.getMatrixFromQuaternion(cur_quat)).reshape(3, 3)
        thrust_body = np.dot(cur_rotation, target_force)
        return thrust_body[2], target_rpy, vel_e

    def _attitudePID(self, dt, thrust, cur_quat, target_rpy):
        cur_rpy = np.array(p.getEulerFromQuaternion(cur_quat)).reshape(3)
        rpy_e = target_rpy - cur_rpy
        rpy_e[2] = (rpy_e[2] + np.pi) % (2 * np.pi) - np.pi
        d_rpy_e = (rpy_e - self.last_rpy_e) / dt
        self.last_rpy_e = rpy_e
        self.integral_rpy_e += rpy_e * dt

        target_torques = (
            np.multiply(self.P_COEFF_TOR, rpy_e)
            + np.multiply(self.I_COEFF_TOR, self.integral_rpy_e)
            + np.multiply(self.D_COEFF_TOR, d_rpy_e)
        )
        return nnlsRPM(
            thrust=thrust,
            x_torque=target_torques[0],
            y_torque=target_torques[1],
            z_torque=target_torques[2],
            counter=self.control_counter,
            max_thrust=self.MAX_THRUST,
            max_xy_torque=self.MAX_XY_TORQUE,
            max_z_torque=self.MAX_Z_TORQUE,
            a=self.A, inv_a=self.INV_A, b_coeff=self.B_COEFF,
        )
