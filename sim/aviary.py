"""Simplified PyBullet drone environment for IFDS integration.

Adapted from ``gym-pybullet-drones-routing/gym_pybullet_drones/envs/BaseAviary.py``.
This is **not** a ``gym.Env`` — it is a lightweight physics runner that loads a
drone URDF, steps the simulation, and exposes the drone state.
"""

from __future__ import annotations

import os
import time
import xml.etree.ElementTree as etxml
from datetime import datetime
from pathlib import Path

import numpy as np
import pybullet as p
import pybullet_data

from sim.enums import DroneModel, Physics

ASSETS_DIR = Path(__file__).resolve().parent / "assets"


class Aviary:
    """Single-drone PyBullet physics environment.

    Parameters
    ----------
    drone_model : DroneModel
        Which URDF to load.
    initial_xyz : ndarray, optional
        (3,)-shaped starting position.  Defaults to ``[0, 0, 0.5]``.
    initial_rpy : ndarray, optional
        (3,)-shaped starting orientation (radians).  Defaults to zeros.
    physics : Physics
        Physics mode (PYB, DYN, PYB_GND, …).
    pyb_freq : int
        PyBullet internal frequency (Hz).
    ctrl_freq : int
        Control frequency (Hz).  Must divide ``pyb_freq``.
    gui : bool
        Open the PyBullet GUI window.
    global_scaling : float
        URDF visual scaling factor applied when loading the drone.
    """

    G = 9.8

    # ------------------------------------------------------------------

    def __init__(
        self,
        drone_model: DroneModel = DroneModel.CF2X,
        initial_xyz: np.ndarray | None = None,
        initial_rpy: np.ndarray | None = None,
        physics: Physics = Physics.PYB,
        pyb_freq: int = 240,
        ctrl_freq: int = 240,
        gui: bool = False,
        global_scaling: float = 2.0,
    ):
        # ---- frequencies ------------------------------------------------
        self.PYB_FREQ = pyb_freq
        self.CTRL_FREQ = ctrl_freq
        if self.PYB_FREQ % self.CTRL_FREQ != 0:
            raise ValueError("pyb_freq must be divisible by ctrl_freq")
        self.PYB_STEPS_PER_CTRL = int(self.PYB_FREQ / self.CTRL_FREQ)
        self.CTRL_TIMESTEP = 1.0 / self.CTRL_FREQ
        self.PYB_TIMESTEP = 1.0 / self.PYB_FREQ

        # ---- options ----------------------------------------------------
        self.DRONE_MODEL = drone_model
        self.GUI = gui
        self.PHYSICS = physics
        self.URDF = self.DRONE_MODEL.value + ".urdf"
        self.GLOBAL_SCALING = global_scaling

        # ---- Parse URDF parameters --------------------------------------
        (
            self.M, self.L, self.THRUST2WEIGHT_RATIO,
            self.J, self.J_INV, self.KF, self.KM,
            self.COLLISION_H, self.COLLISION_R, self.COLLISION_Z_OFFSET,
            self.MAX_SPEED_KMH, self.GND_EFF_COEFF, self.PROP_RADIUS,
            self.DRAG_COEFF, self.DW_COEFF_1, self.DW_COEFF_2, self.DW_COEFF_3,
        ) = self._parseURDFParameters()

        # ---- Derived constants ------------------------------------------
        self.GRAVITY = self.G * self.M
        self.HOVER_RPM = np.sqrt(self.GRAVITY / (4 * self.KF))
        self.MAX_RPM = np.sqrt((self.THRUST2WEIGHT_RATIO * self.GRAVITY) / (4 * self.KF))
        self.MAX_THRUST = 4 * self.KF * self.MAX_RPM ** 2
        if self.DRONE_MODEL in (DroneModel.CF2X, DroneModel.RACE):
            self.MAX_XY_TORQUE = (2 * self.L * self.KF * self.MAX_RPM ** 2) / np.sqrt(2)
        elif self.DRONE_MODEL == DroneModel.CF2P:
            self.MAX_XY_TORQUE = self.L * self.KF * self.MAX_RPM ** 2
        else:
            self.MAX_XY_TORQUE = (2 * self.L * self.KF * self.MAX_RPM ** 2) / np.sqrt(2)
        self.MAX_Z_TORQUE = 2 * self.KM * self.MAX_RPM ** 2

        # ---- Initial poses ----------------------------------------------
        self.INIT_XYZ = np.array(initial_xyz if initial_xyz is not None else [0, 0, 0.5], dtype=float)
        self.INIT_RPY = np.array(initial_rpy if initial_rpy is not None else [0, 0, 0], dtype=float)

        # ---- Connect to PyBullet ----------------------------------------
        if self.GUI:
            self.CLIENT = p.connect(p.GUI)
            for flag in [
                p.COV_ENABLE_RGB_BUFFER_PREVIEW,
                p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,
                p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,
            ]:
                p.configureDebugVisualizer(flag, 0, physicsClientId=self.CLIENT)
            p.resetDebugVisualizerCamera(
                cameraDistance=3, cameraYaw=-30, cameraPitch=-30,
                cameraTargetPosition=[0, 0, 0], physicsClientId=self.CLIENT,
            )
        else:
            self.CLIENT = p.connect(p.DIRECT)

        # ---- Housekeeping (loads plane + drone) -------------------------
        self._housekeeping()

    # ------------------------------------------------------------------
    # Housekeeping
    # ------------------------------------------------------------------

    def _housekeeping(self):
        """Reset simulation state, load ground plane and drone URDF."""
        self.RESET_TIME = time.time()
        self.step_counter = 0
        self.last_clipped_action = np.zeros(4)

        # ---- Kinematic state arrays (single drone) ----------------------
        self.pos = np.zeros(3)
        self.quat = np.zeros(4)
        self.rpy = np.zeros(3)
        self.vel = np.zeros(3)
        self.ang_v = np.zeros(3)
        if self.PHYSICS == Physics.DYN:
            self.rpy_rates = np.zeros(3)

        # ---- PyBullet sim parameters ------------------------------------
        p.resetSimulation(physicsClientId=self.CLIENT)
        p.setGravity(0, 0, -self.G, physicsClientId=self.CLIENT)
        p.setRealTimeSimulation(0, physicsClientId=self.CLIENT)
        p.setTimeStep(self.PYB_TIMESTEP, physicsClientId=self.CLIENT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.CLIENT)

        # ---- Ground plane -----------------------------------------------
        # Use a custom .obj mesh with UV coords (0,0)-(1,1) spanning the
        # full IFDS domain so the weather texture maps 1:1 (no tiling).
        ground_mesh = str(ASSETS_DIR / "ground_plane.obj")
        col_id = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[100.0, 100.0, 0.005],
            collisionFramePosition=[100.0, 0.0, -0.005],
            physicsClientId=self.CLIENT,
        )
        vis_id = p.createVisualShape(
            p.GEOM_MESH,
            fileName=ground_mesh,
            rgbaColor=[0.85, 0.85, 0.85, 1.0],
            physicsClientId=self.CLIENT,
        )
        self.PLANE_ID = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=col_id,
            baseVisualShapeIndex=vis_id,
            basePosition=[0, 0, 0],
            physicsClientId=self.CLIENT,
        )

        # ---- Drone URDF -------------------------------------------------
        urdf_path = str(ASSETS_DIR / self.URDF)
        self.DRONE_ID = p.loadURDF(
            urdf_path,
            self.INIT_XYZ,
            p.getQuaternionFromEuler(self.INIT_RPY),
            flags=p.URDF_USE_INERTIA_FROM_FILE,
            globalScaling=self.GLOBAL_SCALING,
            physicsClientId=self.CLIENT,
        )

        # ---- Debug axes (GUI only) --------------------------------------
        self._X_AX = -1
        self._Y_AX = -1
        self._Z_AX = -1
        if self.GUI:
            self._showDroneLocalAxes()

        # ---- Sync state --------------------------------------------------
        self._updateKinematics()

    # ------------------------------------------------------------------
    # State accessors
    # ------------------------------------------------------------------

    def _updateKinematics(self):
        """Pull position, orientation, velocity from PyBullet."""
        self.pos, self.quat = np.array(p.getBasePositionAndOrientation(
            self.DRONE_ID, physicsClientId=self.CLIENT
        ), dtype=object)
        self.pos = np.array(self.pos, dtype=float)
        self.quat = np.array(self.quat, dtype=float)
        self.rpy = np.array(p.getEulerFromQuaternion(self.quat), dtype=float)
        lin, ang = p.getBaseVelocity(self.DRONE_ID, physicsClientId=self.CLIENT)
        self.vel = np.array(lin, dtype=float)
        self.ang_v = np.array(ang, dtype=float)

    def get_state(self) -> np.ndarray:
        """Return the 20-element state vector (same layout as gym-pybullet-drones).

        ``[x, y, z, qx, qy, qz, qw, r, p, y, vx, vy, vz, wx, wy, wz, rpm0..3]``
        """
        return np.hstack([
            self.pos, self.quat, self.rpy,
            self.vel, self.ang_v, self.last_clipped_action,
        ])

    # ------------------------------------------------------------------
    # Step
    # ------------------------------------------------------------------

    def step(self, rpm: np.ndarray):
        """Advance the simulation by one control step.

        Parameters
        ----------
        rpm : ndarray
            (4,)-shaped array of motor RPMs.
        """
        clipped = np.copy(rpm)
        for _ in range(self.PYB_STEPS_PER_CTRL):
            if self.PYB_STEPS_PER_CTRL > 1 and self.PHYSICS in (
                Physics.DYN, Physics.PYB_GND, Physics.PYB_DRAG,
                Physics.PYB_DW, Physics.PYB_GND_DRAG_DW,
            ):
                self._updateKinematics()

            if self.PHYSICS == Physics.PYB:
                self._physics(clipped)
            elif self.PHYSICS == Physics.DYN:
                self._dynamics(clipped)
            elif self.PHYSICS == Physics.PYB_GND:
                self._physics(clipped)
                self._groundEffect(clipped)
            elif self.PHYSICS == Physics.PYB_DRAG:
                self._physics(clipped)
                self._drag(self.last_clipped_action)
            elif self.PHYSICS == Physics.PYB_DW:
                self._physics(clipped)
            elif self.PHYSICS == Physics.PYB_GND_DRAG_DW:
                self._physics(clipped)
                self._groundEffect(clipped)
                self._drag(self.last_clipped_action)

            if self.PHYSICS != Physics.DYN:
                p.stepSimulation(physicsClientId=self.CLIENT)

            self.last_clipped_action = clipped

        self._updateKinematics()
        self.step_counter += self.PYB_STEPS_PER_CTRL

    # ------------------------------------------------------------------
    # Physics helpers
    # ------------------------------------------------------------------

    def _physics(self, rpm: np.ndarray):
        """Apply motor forces/torques (base PyBullet physics)."""
        forces = np.array(rpm ** 2) * self.KF
        torques = np.array(rpm ** 2) * self.KM
        if self.DRONE_MODEL == DroneModel.RACE:
            torques = -torques
        z_torque = -torques[0] + torques[1] - torques[2] + torques[3]
        for i in range(4):
            p.applyExternalForce(
                self.DRONE_ID, i,
                forceObj=[0, 0, forces[i]],
                posObj=[0, 0, 0],
                flags=p.LINK_FRAME,
                physicsClientId=self.CLIENT,
            )
        p.applyExternalTorque(
            self.DRONE_ID, 4,
            torqueObj=[0, 0, z_torque],
            flags=p.LINK_FRAME,
            physicsClientId=self.CLIENT,
        )

    def _dynamics(self, rpm: np.ndarray):
        """Explicit dynamics integration (from BaseAviary._dynamics)."""
        pos = self.pos
        quat = self.quat
        vel = self.vel
        rpy_rates = self.rpy_rates
        rotation = np.array(p.getMatrixFromQuaternion(quat)).reshape(3, 3)

        forces = np.array(rpm ** 2) * self.KF
        thrust = np.array([0, 0, np.sum(forces)])
        thrust_world = np.dot(rotation, thrust)
        force_world = thrust_world - np.array([0, 0, self.GRAVITY])

        z_torques = np.array(rpm ** 2) * self.KM
        if self.DRONE_MODEL == DroneModel.RACE:
            z_torques = -z_torques
        z_torque = -z_torques[0] + z_torques[1] - z_torques[2] + z_torques[3]

        if self.DRONE_MODEL in (DroneModel.CF2X, DroneModel.RACE):
            x_torque = (forces[0] + forces[1] - forces[2] - forces[3]) * (self.L / np.sqrt(2))
            y_torque = (-forces[0] + forces[1] + forces[2] - forces[3]) * (self.L / np.sqrt(2))
        elif self.DRONE_MODEL == DroneModel.CF2P:
            x_torque = (forces[1] - forces[3]) * self.L
            y_torque = (-forces[0] + forces[2]) * self.L
        else:
            x_torque = (forces[0] + forces[1] - forces[2] - forces[3]) * (self.L / np.sqrt(2))
            y_torque = (-forces[0] + forces[1] + forces[2] - forces[3]) * (self.L / np.sqrt(2))

        torq = np.array([x_torque, y_torque, z_torque])
        torq = torq - np.cross(rpy_rates, np.dot(self.J, rpy_rates))
        rpy_rates_deriv = np.dot(self.J_INV, torq)
        accs = force_world / self.M

        vel = vel + self.PYB_TIMESTEP * accs
        rpy_rates = rpy_rates + self.PYB_TIMESTEP * rpy_rates_deriv
        pos = pos + self.PYB_TIMESTEP * vel
        quat = self._integrateQ(quat, rpy_rates, self.PYB_TIMESTEP)

        p.resetBasePositionAndOrientation(self.DRONE_ID, pos, quat, physicsClientId=self.CLIENT)
        p.resetBaseVelocity(self.DRONE_ID, vel, np.dot(rotation, rpy_rates), physicsClientId=self.CLIENT)
        self.rpy_rates = rpy_rates

    def _groundEffect(self, rpm: np.ndarray):
        """Simple per-propeller ground effect model."""
        link_states = p.getLinkStates(
            self.DRONE_ID, linkIndices=[0, 1, 2, 3, 4],
            computeLinkVelocity=1, computeForwardKinematics=1,
            physicsClientId=self.CLIENT,
        )
        prop_heights = np.array([link_states[i][0][2] for i in range(4)])
        clip_h = 0.25 * self.PROP_RADIUS * np.sqrt(
            (15 * self.MAX_RPM ** 2 * self.KF * self.GND_EFF_COEFF) / self.MAX_THRUST
        )
        prop_heights = np.clip(prop_heights, clip_h, np.inf)
        gnd = np.array(rpm ** 2) * self.KF * self.GND_EFF_COEFF * (self.PROP_RADIUS / (4 * prop_heights)) ** 2
        if np.abs(self.rpy[0]) < np.pi / 2 and np.abs(self.rpy[1]) < np.pi / 2:
            for i in range(4):
                p.applyExternalForce(
                    self.DRONE_ID, i,
                    forceObj=[0, 0, gnd[i]], posObj=[0, 0, 0],
                    flags=p.LINK_FRAME, physicsClientId=self.CLIENT,
                )

    def _drag(self, rpm: np.ndarray):
        """Simple drag model (Forster, 2015)."""
        base_rot = np.array(p.getMatrixFromQuaternion(self.quat)).reshape(3, 3)
        drag_factors = -1 * self.DRAG_COEFF * np.sum(2 * np.pi * rpm / 60)
        drag = np.dot(base_rot.T, drag_factors * self.vel)
        p.applyExternalForce(
            self.DRONE_ID, 4,
            forceObj=drag, posObj=[0, 0, 0],
            flags=p.LINK_FRAME, physicsClientId=self.CLIENT,
        )

    @staticmethod
    def _integrateQ(quat, omega, dt):
        """Quaternion integration from body-frame angular velocity."""
        omega_norm = np.linalg.norm(omega)
        px, qx, rx = omega
        if np.isclose(omega_norm, 0):
            return quat
        lam = np.array([
            [0,   rx, -qx, px],
            [-rx,  0,  px, qx],
            [qx, -px,  0,  rx],
            [-px, -qx, -rx, 0],
        ]) * 0.5
        theta = omega_norm * dt / 2
        return np.dot(np.eye(4) * np.cos(theta) + 2 / omega_norm * lam * np.sin(theta), quat)

    # ------------------------------------------------------------------
    # URDF parsing
    # ------------------------------------------------------------------

    def _parseURDFParameters(self):
        """Load drone parameters from the local URDF file."""
        urdf_path = ASSETS_DIR / self.URDF
        tree = etxml.parse(str(urdf_path)).getroot()
        M = float(tree[1][0][1].attrib['value'])
        L = float(tree[0].attrib['arm'])
        THRUST2WEIGHT_RATIO = float(tree[0].attrib['thrust2weight'])
        IXX = float(tree[1][0][2].attrib['ixx'])
        IYY = float(tree[1][0][2].attrib['iyy'])
        IZZ = float(tree[1][0][2].attrib['izz'])
        J = np.diag([IXX, IYY, IZZ])
        J_INV = np.linalg.inv(J)
        KF = float(tree[0].attrib['kf'])
        KM = float(tree[0].attrib['km'])
        COLLISION_H = float(tree[1][2][1][0].attrib['length'])
        COLLISION_R = float(tree[1][2][1][0].attrib['radius'])
        offsets = [float(s) for s in tree[1][2][0].attrib['xyz'].split(' ')]
        COLLISION_Z_OFFSET = offsets[2]
        MAX_SPEED_KMH = float(tree[0].attrib['max_speed_kmh'])
        GND_EFF_COEFF = float(tree[0].attrib['gnd_eff_coeff'])
        PROP_RADIUS = float(tree[0].attrib['prop_radius'])
        DRAG_XY = float(tree[0].attrib['drag_coeff_xy'])
        DRAG_Z = float(tree[0].attrib['drag_coeff_z'])
        DRAG_COEFF = np.array([DRAG_XY, DRAG_XY, DRAG_Z])
        DW1 = float(tree[0].attrib['dw_coeff_1'])
        DW2 = float(tree[0].attrib['dw_coeff_2'])
        DW3 = float(tree[0].attrib['dw_coeff_3'])
        return (M, L, THRUST2WEIGHT_RATIO, J, J_INV, KF, KM,
                COLLISION_H, COLLISION_R, COLLISION_Z_OFFSET,
                MAX_SPEED_KMH, GND_EFF_COEFF, PROP_RADIUS,
                DRAG_COEFF, DW1, DW2, DW3)

    # ------------------------------------------------------------------
    # Visualisation helpers
    # ------------------------------------------------------------------

    def _showDroneLocalAxes(self):
        """Draw RGB axes on the drone in the GUI."""
        axis_len = 2 * self.L
        self._X_AX = p.addUserDebugLine(
            [0, 0, 0], [axis_len, 0, 0], [1, 0, 0],
            parentObjectUniqueId=self.DRONE_ID, parentLinkIndex=-1,
            replaceItemUniqueId=int(self._X_AX), physicsClientId=self.CLIENT,
        )
        self._Y_AX = p.addUserDebugLine(
            [0, 0, 0], [0, axis_len, 0], [0, 1, 0],
            parentObjectUniqueId=self.DRONE_ID, parentLinkIndex=-1,
            replaceItemUniqueId=int(self._Y_AX), physicsClientId=self.CLIENT,
        )
        self._Z_AX = p.addUserDebugLine(
            [0, 0, 0], [0, 0, axis_len], [0, 0, 1],
            parentObjectUniqueId=self.DRONE_ID, parentLinkIndex=-1,
            replaceItemUniqueId=int(self._Z_AX), physicsClientId=self.CLIENT,
        )

    def set_camera(self, target: np.ndarray, distance: float = 5.0,
                   yaw: float = -30, pitch: float = -30):
        """Reposition the debug-visualiser camera (GUI only)."""
        if self.GUI:
            p.resetDebugVisualizerCamera(
                cameraDistance=distance, cameraYaw=yaw, cameraPitch=pitch,
                cameraTargetPosition=target.tolist(),
                physicsClientId=self.CLIENT,
            )

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def reset(self):
        """Reset simulation to initial state."""
        self._housekeeping()

    def close(self):
        """Disconnect from the PyBullet server."""
        p.disconnect(physicsClientId=self.CLIENT)
