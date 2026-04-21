"""PyBullet simulation runner for IFDS dynamic autorouting.

This module provides :func:`run_pybullet_segment` (single IFDS segment) and
:func:`run_pybullet` (full outer-loop replacement), driving a drone through
IFDS waypoints using **velocity-based PID control** — the same approach used
in ``gym-pybullet-drones-routing``.

The guidance layer (waypoint skipping → target velocity) is ported from
``IFDSRoute._waypointSkipping`` + ``_updateTargetVel``.

Weather is overlaid as a ground-plane texture when a :class:`WeatherField` is
provided.
"""

from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import pybullet as p

from sim.aviary import Aviary
from sim.control import PIDVelocityControl
from sim.enums import DroneModel, Physics
from sim.weather_texture import WeatherGroundPlane

from ifds.scenes import build_scene


# ======================================================================
# Helpers
# ======================================================================

# Obstacle shape colour palette (solid grey)
_OBS_COLORS = [
    [0.6, 0.6, 0.6, 1.0],
]


def _classify_shape(p_exp: float, q_exp: float, r_exp: float) -> str:
    """Map IFDS exponents to a shape class name."""
    if abs(r_exp - 0.5) < 0.1:
        return "cone"
    if abs(r_exp - 4.0) < 0.5:
        return "cylinder"
    if abs(p_exp - 1) < 0.1 and abs(q_exp - 1) < 0.1 and abs(r_exp - 1) < 0.1:
        return "sphere"
    if p_exp >= 10:
        return "box"   # ceiling
    return "cylinder"  # pipe and others → cylinder approximation


def _create_obstacle_bodies(
    objects, client: int,
) -> list[int]:
    """Create visual-only PyBullet bodies for each IFDS obstacle.

    Returns a list of body IDs (one per obstacle).
    """
    body_ids: list[int] = []
    for idx, obj in enumerate(objects):
        if obj.a == 0 and obj.b == 0 and obj.c == 0:
            body_ids.append(-1)
            continue

        rgba = _OBS_COLORS[idx % len(_OBS_COLORS)]
        shape_class = _classify_shape(obj.p, obj.q, obj.r)
        x0, y0, z0 = obj.origin

        if shape_class == "sphere":
            vis = p.createVisualShape(
                p.GEOM_SPHERE, radius=obj.a,
                rgbaColor=rgba, physicsClientId=client,
            )
            pos = [x0, y0, z0]
        elif shape_class == "cylinder":
            half_h = obj.c
            vis = p.createVisualShape(
                p.GEOM_CYLINDER, radius=obj.a, length=2 * half_h,
                rgbaColor=rgba, physicsClientId=client,
            )
            pos = [x0, y0, z0 + half_h]  # PyBullet cylinder centered at mid-height
        elif shape_class == "cone":
            # PyBullet has no cone primitive — approximate with a mesh-less
            # cylinder that tapers.  Use a slim cylinder as a stand-in.
            half_h = obj.c
            vis = p.createVisualShape(
                p.GEOM_CYLINDER, radius=obj.a * 0.6, length=2 * half_h,
                rgbaColor=rgba, physicsClientId=client,
            )
            pos = [x0, y0, z0 + half_h]
        elif shape_class == "box":
            vis = p.createVisualShape(
                p.GEOM_BOX, halfExtents=[obj.a, obj.b, obj.c],
                rgbaColor=rgba, physicsClientId=client,
            )
            pos = [x0, y0, z0]
        else:
            body_ids.append(-1)
            continue

        bid = p.createMultiBody(
            baseMass=0, baseVisualShapeIndex=vis,
            basePosition=pos, physicsClientId=client,
        )
        body_ids.append(bid)
    return body_ids


def _update_obstacle_positions(
    objects, body_ids: list[int], client: int,
) -> None:
    """Move existing obstacle bodies to match updated IFDS object origins."""
    for idx, (obj, bid) in enumerate(zip(objects, body_ids)):
        if bid < 0:
            continue
        x0, y0, z0 = obj.origin
        shape_class = _classify_shape(obj.p, obj.q, obj.r)
        if shape_class in ("cylinder", "cone"):
            pos = [x0, y0, z0 + obj.c]
        else:
            pos = [x0, y0, z0]
        p.resetBasePositionAndOrientation(
            bid, pos, [0, 0, 0, 1], physicsClientId=client,
        )


_DRONE_MODEL_MAP: dict[str, DroneModel] = {
    "cf2x": DroneModel.CF2X,
    "cf2p": DroneModel.CF2P,
    "hb": DroneModel.HB,
    "racer": DroneModel.RACE,
}


def _resolve_drone_model(name: str) -> DroneModel:
    key = name.lower().replace("-", "").replace("_", "")
    if key in _DRONE_MODEL_MAP:
        return _DRONE_MODEL_MAP[key]
    raise ValueError(f"Unknown drone model '{name}'. Choose from: {list(_DRONE_MODEL_MAP)}")


# ======================================================================
# Waypoint-skipping guidance (ported from IFDSRoute)
# ======================================================================

def _waypoint_skip(path: np.ndarray, cur_pos: np.ndarray,
                   cur_vel: np.ndarray, speed_limit: float,
                   dt: float, wp_idx_min: int = 2,
                   wp_closeness: float = 1.0,
                   ) -> tuple[np.ndarray, int]:
    """Port of ``IFDSRoute._waypointSkipping`` + ``_updateTargetVel``.

    Given a waypoint path ``(3, N)`` and the drone's current state, compute
    a **target velocity vector** that steers the drone along the path at up
    to *speed_limit* m/s.

    Parameters
    ----------
    path : ndarray (3, N)
    cur_pos, cur_vel : ndarray (3,)
    speed_limit : float   m/s
    dt : float            control timestep
    wp_idx_min : int      earliest waypoint index to consider (monotonically
                          increases across calls to prevent loopback)
    wp_closeness : float  distance to consider a waypoint "reached"

    Returns
    -------
    target_vel : ndarray (3,)
    new_wp_idx_min : int   updated minimum waypoint index for next call
    """
    n_wp = path.shape[1]
    if n_wp == 0:
        return np.zeros(3), wp_idx_min
    if n_wp == 1:
        diff = path[:, 0] - cur_pos
        norm = np.linalg.norm(diff)
        if norm > 1e-6:
            return diff / norm * min(norm, speed_limit), wp_idx_min
        return np.zeros(3), wp_idx_min

    # Never go backwards — start from the remembered minimum index
    k = min(max(wp_idx_min, 2), n_wp - 1)
    Wf = path[:, k]

    while True:
        if k >= n_wp - 1:
            break
        k_n = min(k + 1, n_wp - 1)
        Wi = path[:, k]
        Wf = path[:, k_n]

        path_vect = Wf - Wi
        norm_pv = np.linalg.norm(path_vect)
        if norm_pv < 1e-9:
            break

        a, b, c = path_vect
        # Check if waypoint is still ahead of the drone (dot-product test)
        if a * (cur_pos[0] - Wf[0]) + b * (cur_pos[1] - Wf[1]) + c * (cur_pos[2] - Wf[2]) < 0:
            # Wf is still ahead
            if np.linalg.norm(cur_pos - Wf) <= wp_closeness:
                k += 1
            else:
                break
        else:
            # Wf is behind us — skip to next
            k += 1

    # Direction from current position to the target waypoint
    diff = Wf - cur_pos
    dist = np.linalg.norm(diff)
    if dist < 1e-6:
        return np.zeros(3), k
    unit = diff / dist

    # Compute speed: accelerate towards speed_limit, matching the source repo
    cur_speed = float(np.linalg.norm(cur_vel))
    cur_speed = min(cur_speed, speed_limit)
    accel = 5.0  # m/s^2
    new_speed = min(cur_speed + accel * dt, speed_limit)

    target_vel = new_speed * unit

    # Final clamp
    if np.linalg.norm(target_vel) > speed_limit:
        target_vel = target_vel / np.linalg.norm(target_vel) * speed_limit

    return target_vel, k


# ======================================================================
# Single-segment follower
# ======================================================================

def run_pybullet_segment(
    aviary: Aviary,
    ctrl: PIDVelocityControl,
    path: np.ndarray,
    *,
    speed_limit: float = 10.0,
    max_steps: int = 5000,
    target_thresh: float = 2.0,
    on_tick=None,
) -> dict:
    """Follow a waypoint path using velocity-PID control inside the aviary.

    This mirrors the control loop in ``ExtendedSARLAviary._preprocessAction``:
    guidance produces ``target_vel``, then ``PIDVelocityControl.computeControl``
    turns that into motor RPMs.

    Parameters
    ----------
    aviary : Aviary
        An already-initialised aviary.
    ctrl : PIDVelocityControl
        Velocity PID controller.
    path : ndarray
        ``(3, N)`` array of waypoints (columns are [x, y, z]).
    speed_limit : float
        Maximum drone speed in m/s.
    max_steps : int
        Safety limit on control iterations.
    target_thresh : float
        Distance to the final waypoint that counts as "reached".
    on_tick : callable | None
        ``on_tick(step_i, max_steps)`` called every control step for
        continuous obstacle / weather updates during flight.

    Returns
    -------
    dict
        ``pos_hist`` (list of (3,) arrays), ``quat_hist``.
    """
    pos_hist: list[np.ndarray] = []
    quat_hist: list[np.ndarray] = []

    final_pos = path[:, -1]
    dt = aviary.CTRL_TIMESTEP

    # Persistent waypoint index — prevents the drone from looping back
    wp_idx_min = 2

    for step_i in range(max_steps):
        state = aviary.get_state()
        cur_pos = state[0:3]
        cur_quat = state[3:7]
        cur_vel = state[10:13]
        cur_ang_vel = state[13:16]

        pos_hist.append(cur_pos.copy())
        quat_hist.append(cur_quat.copy())

        # Check arrival at final waypoint
        if np.linalg.norm(cur_pos - final_pos) < target_thresh:
            break

        # Guidance: waypoint skipping → target velocity
        target_vel, wp_idx_min = _waypoint_skip(
            path, cur_pos, cur_vel,
            speed_limit=speed_limit, dt=dt,
            wp_idx_min=wp_idx_min,
        )

        # PID velocity control → motor RPMs
        rpm, _, _ = ctrl.computeControl(
            control_timestep=dt,
            cur_pos=cur_pos,
            cur_quat=cur_quat,
            cur_vel=cur_vel,
            cur_ang_vel=cur_ang_vel,
            target_vel=target_vel,
        )

        aviary.step(rpm)

        # Periodic callback for obstacle / weather updates
        if on_tick is not None:
            on_tick(step_i, max_steps)

    return dict(
        pos_hist=pos_hist,
        quat_hist=quat_hist,
    )


# ======================================================================
# Full outer-loop runner
# ======================================================================

def run_pybullet(
    param,
    paths: list[list[np.ndarray | None]],
    objects_per_rt,
    destin: np.ndarray,
    *,
    weather=None,
    drone_model_name: str = "cf2x",
    gui: bool = True,
    ctrl_freq: int = 60,
    pyb_freq: int = 60,
    physics: str = "pyb",
    global_scaling: float = 1.0,
    speed_limit: float = 0.0,
) -> dict:
    """Run the full IFDS simulation inside PyBullet.

    This replaces the CCA3D / SixDoF inner loop with a PyBullet physics
    simulation.  IFDS paths are assumed to have been pre-computed and are
    passed in via *paths*.

    The flight control uses the **same approach as
    gym-pybullet-drones-routing**: velocity-based waypoint guidance +
    ``PIDVelocityControl`` at matching ``pyb_freq == ctrl_freq`` (default 30 Hz).

    Parameters
    ----------
    param : Param
        IFDS parameters (provides ``x_i, y_i, z_i, psi_i, rtsim``, etc.).
    paths : list[list[ndarray | None]]
        ``paths[line][rt]`` — waypoint array ``(3, N)`` per outer step.
    objects_per_rt
        Objects list (for result compatibility).
    destin : ndarray
        Destination array ``(num_line, 3)``.
    weather : WeatherField | None
        Optional weather field for texture overlay.
    drone_model_name : str
        One of ``cf2x``, ``cf2p``, ``hb``, ``racer``.
    gui : bool
        Open the PyBullet GUI.
    ctrl_freq : int
        PID control rate (Hz).  Default 30 matches the source repo.
    pyb_freq : int
        PyBullet internal step rate (Hz).  Default 30 matches the source repo.
    physics : str
        Physics mode string (``"pyb"``, ``"dyn"``, …).
    global_scaling : float
        URDF visual scale factor.
    speed_limit : float
        Maximum drone speed in m/s.

    Returns
    -------
    dict
        Result dictionary compatible with ``_plot()`` / ``_animate()``.
    """
    drone_model = _resolve_drone_model(drone_model_name)
    phys = Physics(physics)

    # Use the IFDS UAV speed (param.c) when speed_limit not explicitly set
    if speed_limit <= 0:
        speed_limit = getattr(param, 'c', 10.0)

    x_i, y_i, z_i = param.x_i, param.y_i, param.z_i
    psi_i = getattr(param, "psi_i", 0.0)

    aviary = Aviary(
        drone_model=drone_model,
        initial_xyz=np.array([x_i, y_i, z_i]),
        initial_rpy=np.array([0, 0, psi_i]),
        physics=phys,
        pyb_freq=pyb_freq,
        ctrl_freq=ctrl_freq,
        gui=gui,
        global_scaling=global_scaling,
    )

    # Always use PIDVelocityControl (as in gym-pybullet-drones-routing)
    ctrl = PIDVelocityControl(drone_model)

    # Weather ground-plane texture
    weather_gp: WeatherGroundPlane | None = None
    if weather is not None:
        weather_gp = WeatherGroundPlane(aviary.CLIENT, aviary.PLANE_ID)
        # Apply the first frame immediately so the weather is visible from the start
        weather_gp.update(weather, frame=0, force=True)

    # ---- Spawn IFDS obstacle bodies --------------------------------
    # Build the scene once at rt=0 to populate the Object list, then
    # create visual-only PyBullet bodies for each obstacle.
    build_scene(param.scene, objects_per_rt, x_i, y_i, z_i, 0,
                getattr(param, 'alpha_deg', 0.0))
    obs_body_ids: list[int] = []
    if gui:
        obs_body_ids = _create_obstacle_bodies(objects_per_rt, aviary.CLIENT)

    # Set camera to look at the centre of the domain
    if gui:
        centre = np.array([(destin[:, 0].mean() + x_i) / 2,
                           (destin[:, 1].mean() + y_i) / 2,
                           (destin[:, 2].mean() + z_i) / 2])
        aviary.set_camera(centre, distance=max(50, np.linalg.norm(destin[0] - [x_i, y_i, z_i]) * 0.6))

    # ---- Outer loop (one pass per rt step) --------------------------
    traj: list[np.ndarray | None] = [None] * param.rtsim
    traj[0] = np.array([[x_i], [y_i], [z_i]])
    psi_hist: list[float] = [psi_i]
    gamma_hist: list[float] = [0.0]
    quat_hist: list[np.ndarray] = []

    # Debug line tracking
    trail_line_ids: list[int] = []
    path_line_ids: list[int] = []

    for rt in range(param.rtsim):
        cur_pos = aviary.pos.copy()
        final = destin[0]

        if np.linalg.norm(cur_pos - final) < param.target_thresh:
            print(f"[pybullet] Target reached at rt = {rt}")
            break

        path_rt = paths[0][rt]
        if path_rt is None or path_rt.shape[1] < 2:
            # No valid path this step — hold position
            if traj[rt] is None and rt > 0 and traj[rt - 1] is not None:
                traj[rt] = traj[rt - 1][:, -1:].copy()
            continue

        # Draw the IFDS planned path as blue debug lines (permanent).
        # Remove previous segment's path lines first.
        if gui:
            for lid in path_line_ids:
                p.removeUserDebugItem(lid, physicsClientId=aviary.CLIENT)
            path_line_ids.clear()
            if path_rt.shape[1] > 1:
                for k in range(path_rt.shape[1] - 1):
                    lid = p.addUserDebugLine(
                        path_rt[:, k].tolist(),
                        path_rt[:, k + 1].tolist(),
                        lineColorRGB=[0, 0.5, 1],
                        lineWidth=3.0,
                        lifeTime=0,
                        physicsClientId=aviary.CLIENT,
                    )
                    path_line_ids.append(lid)

        # Reset PID controller to avoid integral windup across segments
        ctrl.reset()

        # Compute step budget from path length and speed
        path_len = float(np.sum(np.linalg.norm(np.diff(path_rt, axis=1), axis=0)))
        time_budget = path_len / max(speed_limit * 0.5, 0.1) * 2.0  # generous
        seg_max_steps = max(int(time_budget * ctrl_freq), 500)
        seg_max_steps = min(seg_max_steps, 30000)

        # ---- Build on_tick callback for continuous updates during flight ----
        # Weather frame range for this segment
        n_frames = weather.n_frames if weather is not None else 1
        w_frame_start = int(rt / max(param.rtsim, 1) * n_frames)
        w_frame_end = int((rt + 1) / max(param.rtsim, 1) * n_frames)
        w_frame_start = min(w_frame_start, n_frames - 1)
        w_frame_end = min(w_frame_end, n_frames - 1)

        # Fractional rt range for obstacle interpolation
        rt_start = float(rt)
        rt_end = float(rt + 1)
        alpha_deg = getattr(param, 'alpha_deg', 0.0)
        update_every = max(int(0.5 / aviary.CTRL_TIMESTEP), 1)  # ~0.5s
        _last_obs_rt = [rt_start]  # mutable for closure
        _last_w_frame = [w_frame_start]

        def _on_tick(step_i, total_steps):
            if step_i % update_every != 0:
                return
            frac = step_i / max(total_steps - 1, 1)

            # Update obstacles with interpolated rt
            if obs_body_ids:
                eff_rt = rt_start + frac * (rt_end - rt_start)
                drone_p = aviary.pos
                build_scene(param.scene, objects_per_rt,
                            float(drone_p[0]), float(drone_p[1]),
                            float(drone_p[2]), eff_rt, alpha_deg)
                _update_obstacle_positions(
                    objects_per_rt, obs_body_ids, aviary.CLIENT)

            # Update weather
            if weather_gp is not None and weather is not None:
                frame = int(w_frame_start + frac * (w_frame_end - w_frame_start))
                frame = min(frame, weather.n_frames - 1)
                if frame != _last_w_frame[0]:
                    weather_gp.update(weather, frame=frame, force=True)
                    _last_w_frame[0] = frame

        # Run velocity-PID control through this segment
        seg_result = run_pybullet_segment(
            aviary, ctrl, path_rt,
            speed_limit=speed_limit,
            max_steps=seg_max_steps,
            target_thresh=max(param.target_thresh, 2.0),
            on_tick=_on_tick,
        )

        # Record trajectory
        if seg_result["pos_hist"]:
            positions = np.array(seg_result["pos_hist"]).T  # (3, K)
            traj[rt] = positions

            # Update heading
            last_pos = positions[:, -1]
            x_i, y_i, z_i = float(last_pos[0]), float(last_pos[1]), float(last_pos[2])
            if positions.shape[1] > 1:
                dx = last_pos[0] - positions[0, -2]
                dy = last_pos[1] - positions[1, -2]
                dz = last_pos[2] - positions[2, -2]
                psi_i = float(np.arctan2(dy, dx))
                gamma_i = float(np.arctan2(-dz, np.sqrt(dx ** 2 + dy ** 2)))
            else:
                gamma_i = 0.0
            psi_hist.append(psi_i)
            gamma_hist.append(gamma_i)

            # Quaternion from last PyBullet state
            if seg_result["quat_hist"]:
                quat_hist.append(seg_result["quat_hist"][-1])

            # Draw actual drone trail as red debug lines (subsampled)
            if gui and positions.shape[1] > 1:
                stride = max(1, positions.shape[1] // 200)
                for k in range(0, positions.shape[1] - 1, stride):
                    lid = p.addUserDebugLine(
                        positions[:, k].tolist(),
                        positions[:, k + 1].tolist(),
                        lineColorRGB=[1, 0, 0],
                        lineWidth=2.0,
                        lifeTime=0,
                        physicsClientId=aviary.CLIENT,
                    )
                    trail_line_ids.append(lid)

    # ---- Build result dict (compatible with _plot / _animate) --------
    traj_clean = [t for t in traj if t is not None]
    # Pad traj back to full length for animation compatibility
    traj_padded: list[np.ndarray | None] = [None] * param.rtsim
    for i, t in enumerate(traj_clean):
        if i < param.rtsim:
            traj_padded[i] = t

    attitude: dict = dict(psi=np.array(psi_hist), gamma=np.array(gamma_hist))
    if quat_hist:
        attitude["quat"] = np.array(quat_hist)

    result = dict(
        paths=paths,
        traj=traj_padded,
        objects=objects_per_rt,
        destin=destin,
        timer=np.zeros(param.rtsim),
        errn=[[] for _ in range(param.rtsim)],
        weather=weather,
        attitude=attitude,
    )

    # Keep aviary alive (user can inspect GUI) — attach to result for cleanup
    result["_aviary"] = aviary

    return result
