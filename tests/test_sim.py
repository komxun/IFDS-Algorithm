"""Smoke tests for the sim/ PyBullet integration package.

All tests run PyBullet in DIRECT (headless) mode so no GUI is required.
"""

from __future__ import annotations

import numpy as np
import pytest

from sim.enums import DroneModel, Physics


# ======================================================================
# Enum tests
# ======================================================================

class TestEnums:
    def test_drone_model_values(self):
        assert DroneModel.CF2X.value == "cf2x"
        assert DroneModel.HB.value == "hb"
        assert DroneModel.RACE.value == "racer"

    def test_physics_values(self):
        assert Physics.PYB.value == "pyb"
        assert Physics.DYN.value == "dyn"


# ======================================================================
# Aviary tests
# ======================================================================

class TestAviary:
    def test_init_and_close(self):
        from sim.aviary import Aviary
        env = Aviary(drone_model=DroneModel.CF2X, gui=False)
        state = env.get_state()
        assert state.shape == (20,)
        env.close()

    def test_initial_position(self):
        from sim.aviary import Aviary
        xyz = np.array([10.0, 5.0, 2.0])
        env = Aviary(drone_model=DroneModel.CF2X, initial_xyz=xyz, gui=False)
        np.testing.assert_allclose(env.pos, xyz, atol=0.1)
        env.close()

    def test_step_runs(self):
        from sim.aviary import Aviary
        env = Aviary(drone_model=DroneModel.CF2X, gui=False)
        hover_rpm = np.full(4, env.HOVER_RPM)
        env.step(hover_rpm)
        assert env.step_counter > 0
        env.close()

    def test_hb_model_loads(self):
        from sim.aviary import Aviary
        env = Aviary(drone_model=DroneModel.HB, gui=False)
        assert env.M > 0
        assert env.KF > 0
        env.close()

    def test_urdf_parameters_parsed(self):
        from sim.aviary import Aviary
        env = Aviary(drone_model=DroneModel.CF2X, gui=False)
        assert env.M == pytest.approx(0.027, abs=0.001)
        assert env.KF > 0
        assert env.KM > 0
        assert env.L > 0
        env.close()

    def test_reset(self):
        from sim.aviary import Aviary
        env = Aviary(drone_model=DroneModel.CF2X, gui=False)
        env.step(np.full(4, env.HOVER_RPM))
        env.reset()
        assert env.step_counter == 0
        env.close()

    def test_dyn_physics(self):
        from sim.aviary import Aviary
        env = Aviary(drone_model=DroneModel.CF2X, physics=Physics.DYN, gui=False)
        hover_rpm = np.full(4, env.HOVER_RPM)
        env.step(hover_rpm)
        assert env.step_counter > 0
        env.close()


# ======================================================================
# Control tests
# ======================================================================

class TestControl:
    def test_dsl_pid_instantiation(self):
        from sim.control import DSLPIDControl
        ctrl = DSLPIDControl(DroneModel.CF2X)
        assert ctrl.KF > 0
        assert ctrl.MIXER_MATRIX.shape == (4, 3)

    def test_dsl_pid_compute(self):
        from sim.control import DSLPIDControl
        ctrl = DSLPIDControl(DroneModel.CF2X)
        import pybullet as p
        quat = p.getQuaternionFromEuler([0, 0, 0])
        rpm, pos_e, yaw_e = ctrl.computeControl(
            control_timestep=1 / 48,
            cur_pos=np.array([0, 0, 1.0]),
            cur_quat=np.array(quat),
            cur_vel=np.zeros(3),
            cur_ang_vel=np.zeros(3),
            target_pos=np.array([0, 0, 1.0]),
        )
        assert rpm.shape == (4,)
        assert np.all(rpm > 0)

    def test_velocity_pid_instantiation(self):
        from sim.control import PIDVelocityControl
        ctrl = PIDVelocityControl(DroneModel.HB)
        assert ctrl.KF > 0

    def test_velocity_pid_compute(self):
        from sim.control import PIDVelocityControl
        ctrl = PIDVelocityControl(DroneModel.HB)
        import pybullet as p
        quat = p.getQuaternionFromEuler([0, 0, 0])
        rpm, vel_e, yaw_e = ctrl.computeControl(
            control_timestep=1 / 48,
            cur_pos=np.zeros(3),
            cur_quat=np.array(quat),
            cur_vel=np.zeros(3),
            cur_ang_vel=np.zeros(3),
            target_vel=np.array([1, 0, 0]),
        )
        assert rpm.shape == (4,)

    def test_nnls_rpm(self):
        from sim.control import nnlsRPM
        a = np.array([[1, 1, 1, 1], [0, 1, 0, -1], [-1, 0, 1, 0], [-1, 1, -1, 1]])
        inv_a = np.linalg.inv(a)
        b = np.array([1.0, 1.0, 1.0, 1.0])
        result = nnlsRPM(1.0, 0.0, 0.0, 0.0, 0, 10.0, 5.0, 2.0, a, inv_a, b)
        assert result.shape == (4,)
        assert np.all(result >= 0)


# ======================================================================
# Weather texture tests
# ======================================================================

class TestWeatherTexture:
    def test_weather_to_rgba_shape(self):
        from sim.weather_texture import weather_to_rgba

        class FakeWeather:
            mat = np.random.rand(50, 50, 5)
            b_u = 0.5

        rgba = weather_to_rgba(FakeWeather(), frame=0)
        assert rgba.shape == (50, 50, 4)
        assert rgba.dtype == np.uint8

    def test_weather_to_rgb_shape(self):
        from sim.weather_texture import weather_to_rgb

        class FakeWeather:
            mat = np.random.rand(50, 50, 5)
            b_u = 0.5

        rgb = weather_to_rgb(FakeWeather(), frame=0, resolution=(256, 256))
        assert rgb.shape[2] == 3
        assert rgb.dtype == np.uint8
