"""Tests for uav.dynamics — quaternion helpers, QuadMixer, and SixDoF EOM."""

from __future__ import annotations

import numpy as np
import pytest

from uav.dynamics import (
    IX, IY, IZ, IU, IV, IW, IQW, IQX, IQY, IQZ, IP, IQ, IR,
    QuadMixer,
    SixDoF,
    quat_from_euler,
    quat_mult,
    quat_normalize,
    quat_to_euler,
    quat_to_rotmat,
)
from uav.profiles import QuadRotorProfile, available_profiles, quadrotor_profile


# ---------------------------------------------------------------------------
# Quaternion helpers
# ---------------------------------------------------------------------------

class TestQuatHelpers:
    def test_identity_rotmat(self):
        q = np.array([1.0, 0.0, 0.0, 0.0])
        np.testing.assert_allclose(quat_to_rotmat(q), np.eye(3), atol=1e-14)

    def test_mult_identity(self):
        q = quat_from_euler(0.3, -0.1, 0.7)
        e = np.array([1.0, 0.0, 0.0, 0.0])
        np.testing.assert_allclose(quat_mult(q, e), q, atol=1e-14)
        np.testing.assert_allclose(quat_mult(e, q), q, atol=1e-14)

    def test_euler_round_trip(self):
        roll, pitch, yaw = 0.2, -0.15, 1.3
        q = quat_from_euler(roll, pitch, yaw)
        r2, p2, y2 = quat_to_euler(q)
        np.testing.assert_allclose([r2, p2, y2], [roll, pitch, yaw], atol=1e-12)

    def test_normalize_zero(self):
        q = np.zeros(4)
        result = quat_normalize(q)
        np.testing.assert_allclose(result, [1, 0, 0, 0])

    def test_normalize_preserves_direction(self):
        q = np.array([2.0, 0.0, 0.0, 0.0])
        result = quat_normalize(q)
        np.testing.assert_allclose(result, [1, 0, 0, 0], atol=1e-14)

    def test_rotmat_orthogonal(self):
        q = quat_from_euler(0.5, 0.3, -0.8)
        R = quat_to_rotmat(q)
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-12)
        np.testing.assert_allclose(np.linalg.det(R), 1.0, atol=1e-12)


# ---------------------------------------------------------------------------
# QuadMixer
# ---------------------------------------------------------------------------

class TestQuadMixer:
    def test_uniform_thrust(self):
        mixer = QuadMixer()
        f = np.array([1.0, 1.0, 1.0, 1.0])
        wrench = mixer.mix(f)
        np.testing.assert_allclose(wrench[0], 4.0, atol=1e-12)
        np.testing.assert_allclose(wrench[1:], 0.0, atol=1e-12)

    def test_mix_unmix_round_trip(self):
        mixer = QuadMixer(arm_length=0.3, k_thrust=2e-5, k_torque=3e-7)
        f = np.array([3.0, 1.5, 2.5, 4.0])
        wrench = mixer.mix(f)
        f_back = mixer.unmix(wrench)
        np.testing.assert_allclose(f_back, f, atol=1e-10)

    def test_from_profile(self):
        profile = quadrotor_profile("generic")
        mixer = QuadMixer.from_profile(profile)
        assert mixer.arm_length == profile.arm_length
        assert mixer.k_thrust == profile.k_thrust
        assert mixer.k_torque == profile.k_torque


# ---------------------------------------------------------------------------
# Profiles
# ---------------------------------------------------------------------------

class TestProfiles:
    def test_available_profiles(self):
        names = available_profiles()
        assert "generic" in names
        assert "dji_matrice_100" in names
        assert "crazyflie" in names

    def test_load_all_presets(self):
        for name in available_profiles():
            p = quadrotor_profile(name)
            assert isinstance(p, QuadRotorProfile)
            assert p.mass > 0
            assert p.arm_length > 0

    def test_unknown_raises(self):
        with pytest.raises(KeyError, match="Unknown quadrotor profile"):
            quadrotor_profile("nonexistent")

    def test_hover_rotor_speed(self):
        p = quadrotor_profile("generic")
        omega_h = p.hover_rotor_speed
        # 4 rotors at hover: 4 * k_thrust * omega^2 = m*g
        total = 4 * p.k_thrust * omega_h**2
        np.testing.assert_allclose(total, p.weight, rtol=1e-10)


# ---------------------------------------------------------------------------
# SixDoF EOM
# ---------------------------------------------------------------------------

class TestSixDoF:
    def _make(self) -> SixDoF:
        return SixDoF()

    def test_hover_holds_position(self):
        """With hover thrust, a level quad should stay put."""
        dyn = self._make()
        s = SixDoF.make_state(x=0, y=0, z=10)
        ctrl = dyn.hover_control()
        dt = 0.01
        for _ in range(100):  # 1 second
            s = dyn.step(s, ctrl, dt)
        np.testing.assert_allclose(s[IX], 0.0, atol=0.01)
        np.testing.assert_allclose(s[IY], 0.0, atol=0.01)
        np.testing.assert_allclose(s[IZ], 10.0, atol=0.01)

    def test_free_fall(self):
        """Zero thrust → object falls under gravity."""
        dyn = self._make()
        s = SixDoF.make_state(z=50.0)
        ctrl = np.zeros(4)
        dt = 0.01
        for _ in range(100):  # 1 second
            s = dyn.step(s, ctrl, dt)
        # After 1 s free fall: z ≈ 50 - 0.5*g*t² ≈ 45.1
        expected_z = 50.0 - 0.5 * dyn.gravity * 1.0**2
        np.testing.assert_allclose(s[IZ], expected_z, atol=0.1)

    def test_quaternion_stays_normalized(self):
        """Quaternion norm should stay ~1 after many steps."""
        dyn = self._make()
        s = SixDoF.make_state(roll=0.3, pitch=-0.1, yaw=0.5)
        ctrl = dyn.hover_control()
        ctrl[1] = 0.001  # small torque perturbation
        dt = 0.01
        for _ in range(500):
            s = dyn.step(s, ctrl, dt)
        q_norm = np.linalg.norm(s[IQW:IQZ + 1])
        np.testing.assert_allclose(q_norm, 1.0, atol=1e-8)

    def test_from_profile(self):
        profile = quadrotor_profile("dji_matrice_100")
        dyn = SixDoF.from_profile(profile)
        assert dyn.mass == profile.mass
        assert dyn.gravity == profile.gravity
        np.testing.assert_array_equal(dyn.inertia, profile.inertia)

    def test_satisfies_dynamics_protocol(self):
        """SixDoF should satisfy the Dynamics Protocol (duck-typing check)."""
        dyn = self._make()
        s = SixDoF.make_state()
        ctrl = dyn.hover_control()
        result = dyn.step(s, ctrl, 0.01)
        assert isinstance(result, np.ndarray)
        assert result.shape == (13,)

    def test_make_state_defaults(self):
        s = SixDoF.make_state()
        assert s.shape == (13,)
        np.testing.assert_allclose(s[:3], 0.0)
        np.testing.assert_allclose(s[IQW], 1.0)
        np.testing.assert_allclose(s[IQX:IQZ + 1], 0.0, atol=1e-14)
