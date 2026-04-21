"""Tests for uav.guidance — L1 guidance, PD attitude controller, and sixdof_follow_segment."""

from __future__ import annotations

import numpy as np
import pytest

from uav.dynamics import SixDoF, quat_from_euler
from uav.guidance import (
    AttitudePDGains,
    L1Params,
    SixDoFResult,
    attitude_pd_control,
    l1_guidance,
    sixdof_follow_segment,
)


# ---------------------------------------------------------------------------
# L1 guidance
# ---------------------------------------------------------------------------

class TestL1Guidance:
    def test_on_track_no_lateral(self):
        """UAV on the segment line, moving along it → near-zero lateral accel."""
        wi = np.array([0.0, 0.0, 50.0])
        wf = np.array([100.0, 0.0, 50.0])
        pos = np.array([30.0, 0.0, 50.0])
        vel = np.array([10.0, 0.0, 0.0])
        a = l1_guidance(pos, vel, wi, wf)
        # Lateral (y) and vertical (z) components should be tiny.
        assert abs(a[1]) < 0.1
        assert abs(a[2]) < 0.5

    def test_off_track_gives_correction(self):
        """UAV offset laterally → guidance produces nonzero y-acceleration."""
        wi = np.array([0.0, 0.0, 50.0])
        wf = np.array([100.0, 0.0, 50.0])
        pos = np.array([30.0, 20.0, 50.0])  # 20 m offset in y
        vel = np.array([10.0, 0.0, 0.0])
        a = l1_guidance(pos, vel, wi, wf)
        assert abs(a[1]) > 0.5  # should be pushing back toward y=0

    def test_zero_length_segment(self):
        """Degenerate segment → zero accel."""
        pt = np.array([10.0, 0.0, 50.0])
        a = l1_guidance(pt, np.array([10.0, 0.0, 0.0]), pt, pt)
        np.testing.assert_allclose(a, 0.0, atol=1e-12)

    def test_stationary_uav(self):
        """Nearly zero speed → returns proportional correction toward ref."""
        wi = np.array([0.0, 0.0, 50.0])
        wf = np.array([100.0, 0.0, 50.0])
        pos = np.array([0.0, 10.0, 50.0])
        vel = np.array([0.0, 0.0, 0.0])
        a = l1_guidance(pos, vel, wi, wf)
        assert np.linalg.norm(a) > 0


# ---------------------------------------------------------------------------
# PD attitude controller
# ---------------------------------------------------------------------------

class TestAttitudePD:
    def test_hover_gives_hover_thrust(self):
        """Level quad, zero desired accel → thrust ≈ weight, zero torques."""
        dyn = SixDoF()
        s = SixDoF.make_state(z=50)
        a_des = np.zeros(3)
        ctrl = attitude_pd_control(s, a_des, 0.0, dyn)
        np.testing.assert_allclose(ctrl[0], dyn.mass * dyn.gravity, rtol=0.05)
        np.testing.assert_allclose(ctrl[1:], 0.0, atol=0.1)

    def test_nonzero_accel_tilts(self):
        """Lateral accel demand → nonzero roll/pitch torque."""
        dyn = SixDoF()
        s = SixDoF.make_state(z=50)
        a_des = np.array([0.0, 2.0, 0.0])  # push in y
        ctrl = attitude_pd_control(s, a_des, 0.0, dyn)
        # At least one torque should be non-trivial.
        assert np.any(np.abs(ctrl[1:3]) > 0.01)


# ---------------------------------------------------------------------------
# sixdof_follow_segment (integration smoke test)
# ---------------------------------------------------------------------------

class TestSixDoFFollowSegment:
    def test_straight_segment_reaches_end(self):
        """Quad should fly roughly along a straight segment and stop past wf."""
        dyn = SixDoF()
        wi = np.array([0.0, 0.0, 50.0])
        wf = np.array([30.0, 0.0, 50.0])
        s0 = SixDoF.make_state(x=0, y=0, z=50, u=10.0)
        res = sixdof_follow_segment(wi, wf, s0, dyn, dt=0.01, max_iter=5000)
        assert isinstance(res, SixDoFResult)
        # Should have crossed past wf in x.
        assert res.x[-1] >= 28.0

    def test_result_has_quat(self):
        """SixDoFResult should expose a quaternion history."""
        dyn = SixDoF()
        wi = np.array([0.0, 0.0, 50.0])
        wf = np.array([10.0, 0.0, 50.0])
        s0 = SixDoF.make_state(x=0, y=0, z=50, u=10.0)
        res = sixdof_follow_segment(wi, wf, s0, dyn, dt=0.01, max_iter=3000)
        assert res.quat.shape[1] == 4
        # Quaternions should be normalized.
        norms = np.linalg.norm(res.quat, axis=1)
        np.testing.assert_allclose(norms, 1.0, atol=1e-6)
