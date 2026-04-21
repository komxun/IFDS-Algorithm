"""Primary entry point — port of ``main.m``.

Runs the full dynamic-autorouting simulation: IFDS path planning per outer step
``rt``, CCA3D path following, trajectory accumulation, and (optional) plotting
and video export.

Usage::

    python -m scripts.run_main --config configs/scene3_static.yaml
    python -m scripts.run_main --config configs/scene3_static.yaml --rho0 0.5
    python -m scripts.run_main --scene 3 --env static   # no config, CLI-only
    python -m scripts.run_main --config c.yaml --dump-config out.yaml
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

from ifds.config import Param
from ifds.config_io import apply_overrides, dump_config, load_config
from ifds.enums import OptimizerMode, Scene
from ifds.ifds import run_ifds
from ifds.optimizer import global_optimize_path, make_global_objective
from ifds.scenes import allocate_objects
from ifds.weather import WeatherField, load_weather_field
from uav.dynamics import SixDoF, quat_to_euler
from uav.guidance import cca3d_straight, sixdof_follow_segment
from uav.profiles import available_profiles, quadrotor_profile

# Lazy import for PyBullet mode — avoid hard dependency when not used.
_run_pybullet = None
def _get_run_pybullet():
    global _run_pybullet
    if _run_pybullet is None:
        from sim.runner import run_pybullet as _rp
        _run_pybullet = _rp
    return _run_pybullet

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_WEATHER = REPO_ROOT / "data" / "WeatherMat_321.mat"


# argparse overrides: defaults are ``None`` so ``apply_overrides`` can tell
# "user didn't set this flag" from "user explicitly set it to the default".
_OVERRIDE_ARG_TO_FIELD: dict[str, str] = {
    "scene": "scene",
    "env": "env",
    "sim_mode": "sim_mode",
    "rtsim": "rtsim",
    "tsim": "tsim",
    "dt": "dt",
    "dt_traj": "dt_traj",
    "rho0": "rho0_initial",
    "sigma0": "sigma0_initial",
    "optimizer": "use_optimizer",
    "delta_g": "rg",
    "sf": "sf",
    "k": "k",
    "bu": "b_u",
    "bl": "b_l",
    "speed": "c",
    "xfinal": "xfinal",
    "yfinal": "yfinal",
    "zfinal": "zfinal",
    "x_i": "x_i",
    "y_i": "y_i",
    "z_i": "z_i",
    "multi_target": "multi_target",
    "cca_preset": "cca_preset",
}


def _build_param(args: argparse.Namespace) -> Param:
    """Compose a ``Param`` from (optional) YAML config + CLI overrides."""
    if args.config is not None:
        param = load_config(args.config)
    else:
        param = Param()

    overrides: dict[str, object] = {
        field: getattr(args, arg_name)
        for arg_name, field in _OVERRIDE_ARG_TO_FIELD.items()
    }
    # Quiet flag maps onto show_disp if the user passed it.
    if args.quiet:
        overrides["show_disp"] = False
    return apply_overrides(param, **overrides)


def _multi_target_destinations() -> np.ndarray:
    return np.array(
        [
            [200, 0, 20], [200, 20, 20], [200, -20, 20],
            [200, 20, 30], [200, -20, 30], [200, 0, 30],
            [200, 0, 40], [200, 20, 40], [200, -20, 40],
        ],
        dtype=float,
    )


def _load_weather(param: Param, path: Path) -> WeatherField | None:
    if param.k == 0:
        return None
    if not path.exists():
        print(f"[warn] weather file {path} not found, disabling weather effects.")
        return None
    return load_weather_field(path, b_l=param.b_l, b_u=param.b_u)


def run(param: Param, *, weather_path: Path = DEFAULT_WEATHER,
        plot: bool = True, animate: bool = False, save_video: Path | None = None,
        dynamics: str = "kinematic", drone_profile: str = "generic",
        drone_model: str = "cf2x", pybullet_gui: bool = True) -> dict:
    """Execute the full dynamic autorouting pipeline and return results."""
    weather = _load_weather(param, weather_path)

    destin = _multi_target_destinations() if param.multi_target \
        else np.array([[param.xfinal, param.yfinal, param.zfinal]], dtype=float)
    num_line = destin.shape[0]
    print(f"Generating paths for {num_line} destinations . . .")
    print("*Timer started*")

    objects = allocate_objects(param.scene)
    wp = np.zeros((3, param.tsim + 1))
    # Paths[line_index][rt] -> np.ndarray of shape (3, N) or None
    paths: list[list[np.ndarray | None]] = [[None] * param.rtsim for _ in range(num_line)]
    traj: list[np.ndarray | None] = [None] * param.rtsim
    traj[0] = np.array([[param.x_i], [param.y_i], [param.z_i]])
    errn: list[list[float]] = [[] for _ in range(param.rtsim)]
    timer = np.zeros(param.rtsim)

    rho0, sigma0 = param.rho0_initial, param.sigma0_initial
    x_i, y_i, z_i = param.x_i, param.y_i, param.z_i
    psi_i, gamma_i = param.psi_i, param.gamma_i
    psi_hist: list[float] = [psi_i]
    gamma_hist: list[float] = [gamma_i]

    use_sixdof = dynamics == "sixdof"
    use_pybullet = dynamics == "pybullet"
    sixdof_dyn: SixDoF | None = None
    sixdof_state: np.ndarray | None = None
    quat_hist: list[np.ndarray] = []
    if use_sixdof:
        profile = quadrotor_profile(drone_profile)
        sixdof_dyn = SixDoF.from_profile(profile)
        sixdof_state = SixDoF.make_state(
            x=x_i, y=y_i, z=z_i, u=param.c, yaw=psi_i, pitch=gamma_i,
        )
        quat_hist.append(sixdof_state[6:10].copy())

    dynamic_scene = param.scene in {Scene.DYNAMIC_3, Scene.DYNAMIC_4, Scene.DYNAMIC_7}

    # ---------- Phase 1: compute IFDS paths for every rt step ----------
    for rt in range(param.rtsim):
        t0 = time.perf_counter()
        if not use_pybullet:
            # For kinematic / sixdof: check arrival from in-loop state
            if np.linalg.norm(np.array([x_i, y_i, z_i])
                              - np.array([param.xfinal, param.yfinal, param.zfinal])) < param.target_thresh:
                print(f"Target destination reached at rt = {rt}")
                traj = [t for t in traj if t is not None]
                break

        if dynamic_scene or (param.k != 0 and param.env == "dynamic"):
            wp[:, 0] = [x_i, y_i, z_i]
        else:
            wp[:, 0] = [param.xini, param.yini, param.zini]

        if traj[rt] is None and rt > 0 and traj[rt - 1] is not None:
            traj[rt] = traj[rt - 1][:, -1:].copy()

        found_path = False
        for line in range(num_line):
            loc_final = destin[line]
            if param.use_optimizer == OptimizerMode.GLOBAL:
                objective = make_global_objective(
                    loc_final=loc_final, rt=rt, wp=wp.copy(),
                    paths=paths, param=param, objects=objects, weather=weather,
                )
                rho0, sigma0 = global_optimize_path(objective, rho0, sigma0)

            paths, objects, _plen, found_path = run_ifds(
                rho0=rho0, sigma0=sigma0, alpha_deg=0.0, loc_final=loc_final,
                rt=rt, wp=wp, paths=paths, param=param, line_index=line,
                objects=objects, weather=weather,
            )

        if not found_path or paths[0][rt] is None or paths[0][rt].shape[1] == 1:
            print(f"CAUTION : Path not found at rt = {rt} s — UAV standing by")
            if use_pybullet:
                continue
            else:
                continue

        # ---- For pybullet mode: paths are collected; skip CCA3D here --
        if use_pybullet:
            timer[rt] = time.perf_counter() - t0
            continue

        # Path following (CCA3D / SixDoF) — slice up to dt_traj of travel
        path_rt = paths[0][rt]
        trajectory = np.zeros((3, path_rt.shape[1]))
        trajectory[:, 0] = [x_i, y_i, z_i]
        i = 0
        dt_cum = 0.0
        err: list[float] = []
        for j in range(path_rt.shape[1] - 1):
            if dt_cum >= param.dt_traj:
                break
            wi = path_rt[:, j]
            wf = path_rt[:, j + 1]
            path_vect = wf - wi
            a, b, c = path_vect
            if a * (x_i - wf[0]) + b * (y_i - wf[1]) + c * (z_i - wf[2]) < 0:
                err.append(float(np.linalg.norm([x_i - wf[0], y_i - wf[1], z_i - wf[2]])))
                if use_sixdof and sixdof_dyn is not None and sixdof_state is not None:
                    res6 = sixdof_follow_segment(
                        wi, wf, sixdof_state, sixdof_dyn, dt=0.01,
                    )
                    sixdof_state = res6.states[-1].copy()
                    x_i = float(res6.x[-1]); y_i = float(res6.y[-1]); z_i = float(res6.z[-1])
                    roll_f, pitch_f, yaw_f = quat_to_euler(sixdof_state[6:10])
                    psi_i = yaw_f; gamma_i = pitch_f
                    psi_hist.append(psi_i); gamma_hist.append(gamma_i)
                    quat_hist.append(sixdof_state[6:10].copy())
                    dt_cum += res6.time_spent
                else:
                    res = cca3d_straight(
                        wi, wf, x_i, y_i, z_i, psi_i, gamma_i, param.c,
                        kappa=param.cca.kappa, delta=param.cca.delta, kd=param.cca.kd,
                    )
                    x_i = float(res.x[-1]); y_i = float(res.y[-1]); z_i = float(res.z[-1])
                    psi_i = float(res.psi[-1]); gamma_i = float(res.gamma[-1])
                    psi_hist.append(psi_i); gamma_hist.append(gamma_i)
                    dt_cum += res.time_spent
                trajectory[:, i + 1] = [x_i, y_i, z_i]
                i += 1
        errn[rt] = err
        trajectory = trajectory[:, : i + 1]
        traj[rt] = trajectory
        timer[rt] = time.perf_counter() - t0
        if param.show_disp:
            print(f"rt={rt}: computed time = {timer[rt]:.4f} s")

    # ---------- Phase 2: if pybullet, run the PyBullet simulation ------
    if use_pybullet:
        print("[pybullet] IFDS paths computed — launching PyBullet simulation . . .")
        run_pyb = _get_run_pybullet()
        result = run_pyb(
            param, paths, objects, destin,
            weather=weather,
            drone_model_name=drone_model,
            gui=pybullet_gui,
        )
        if plot:
            _plot(result, param)
        if animate:
            _animate(result, param)
        if save_video is not None:
            _save_video(result, param, save_video)
        return result

    nz = timer[timer != 0]
    if nz.size:
        print(f"Average computed time = {nz.mean():.4f} s")

    attitude: dict = dict(psi=np.array(psi_hist), gamma=np.array(gamma_hist))
    if use_sixdof and quat_hist:
        attitude["quat"] = np.array(quat_hist)
    result = dict(paths=paths, traj=traj, objects=objects, destin=destin, timer=timer, errn=errn,
                 weather=weather, attitude=attitude)

    if plot:
        _plot(result, param)
    if animate:
        _animate(result, param)
    if save_video is not None:
        _save_video(result, param, save_video)
    return result


def _plot(result: dict, param: Param) -> None:
    import matplotlib.pyplot as plt

    from viz.plotting import _set_equal_aspect_3d, plot_objects_mpl, plot_path_2d, plot_weather_ground
    from viz.uav_marker import draw_arrow, draw_quadrotor

    traj = result["traj"]
    paths = result["paths"]
    objects = result["objects"]
    destin = result["destin"]
    weather = result.get("weather")
    attitude = result.get("attitude")  # dict with psi/gamma or quat

    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection="3d")
    plot_objects_mpl(ax, objects)
    if weather is not None:
        plot_weather_ground(ax, weather, frame=0)
    # draw every generated segment path
    for rt in range(len(traj)):
        if paths[0][rt] is not None:
            wp = paths[0][rt]
            ax.plot(wp[0], wp[1], wp[2], 'b--', alpha=0.4, linewidth=1)
    # full trajectory
    segments = [t for t in traj if t is not None and t.size]
    if segments:
        full = np.concatenate(segments, axis=1)
        ax.plot(full[0], full[1], full[2], 'k', linewidth=1.8, label="UAV Trajectory")
    ax.scatter([param.xini], [param.yini], [param.zini], c='r', marker='o', s=80, label="Start")
    for d in destin:
        ax.scatter([d[0]], [d[1]], [d[2]], c='r', marker='x', s=150)
    # UAV marker at final position
    if segments:
        final_pos = full[:, -1]
        if attitude is not None and "quat" in attitude:
            draw_quadrotor(ax, final_pos, attitude["quat"][-1])
        else:
            psi_f = attitude["psi"][-1] if attitude else 0.0
            gamma_f = attitude["gamma"][-1] if attitude else 0.0
            draw_arrow(ax, final_pos, psi_f, gamma_f)
    ax.set_xlim(0, 200)
    ax.set_ylim(-100, 100)
    ax.set_zlim(0, 100)
    _set_equal_aspect_3d(ax)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title(
        f"IFDS scene={int(param.scene)} ρ₀={param.rho0_initial} σ₀={param.sigma0_initial} "
        f"SF={int(param.sf)} δ_g={param.rg}"
    )
    ax.legend(loc="upper left")
    plt.tight_layout()
    plt.show()


def _animate(result: dict, param: Param) -> None:
    """Show an interactive per-rt animated plot (mirrors MATLAB figure-69 loop)."""
    import matplotlib.pyplot as plt

    from viz.animate import animate_trajectory

    anim = animate_trajectory(
        traj=result["traj"],
        paths=result["paths"],
        objects_per_rt=result["objects"],
        destin=result["destin"],
        weather=result.get("weather"),
        attitude=result.get("attitude"),
        multi_target=param.multi_target,
        xini=param.xini,
        yini=param.yini,
        zini=param.zini,
        fps=2,
    )
    plt.show()


def _save_video(result: dict, param: Param, path: Path) -> None:
    from viz.animate import animate_trajectory

    animate_trajectory(
        traj=result["traj"], paths=result["paths"], objects_per_rt=result["objects"],
        destin=result["destin"], attitude=result.get("attitude"),
        save_path=path, fps=2,
    )
    print(f"Video saved: {path}")


def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="IFDS dynamic autorouting (Python port of main.m)")
    # Config file + dump-config live in their own group so --help makes the
    # two-layer (YAML base + CLI overrides) design obvious.
    cfg = p.add_argument_group("config")
    cfg.add_argument("--config", type=Path, default=None,
                     help="YAML config file to load as the Param base")
    cfg.add_argument("--dump-config", dest="dump_config", type=Path, default=None,
                     help="Write resolved Param to YAML and exit")

    # Every override defaults to None so we can distinguish "unset" from
    # "set to the default" when merging on top of a YAML config.
    ovr = p.add_argument_group("overrides")
    ovr.add_argument("--scene", type=int, default=None,
                     help="Scenario id (0,1,2,3,4,5,7,12,41,42,44,69,6969)")
    ovr.add_argument("--env", choices=["static", "dynamic"], default=None)
    ovr.add_argument("--sim-mode", dest="sim_mode", type=int, default=None, choices=[1, 2])
    ovr.add_argument("--rtsim", type=int, default=None)
    ovr.add_argument("--tsim", type=int, default=None)
    ovr.add_argument("--dt", type=float, default=None)
    ovr.add_argument("--dt-traj", dest="dt_traj", type=float, default=None)
    ovr.add_argument("--rho0", type=float, default=None)
    ovr.add_argument("--sigma0", type=float, default=None)
    ovr.add_argument("--optimizer", type=int, default=None, choices=[0, 1, 2],
                     help="0=off, 1=global, 2=local")
    ovr.add_argument("--delta-g", dest="delta_g", type=float, default=None)
    ovr.add_argument("--sf", type=int, default=None, choices=[0, 1])
    ovr.add_argument("--k", type=float, default=None, help="Weather effect strength (0 disables)")
    ovr.add_argument("--bu", type=float, default=None)
    ovr.add_argument("--bl", type=float, default=None)
    ovr.add_argument("--speed", type=float, default=None)
    ovr.add_argument("--xfinal", type=float, default=None)
    ovr.add_argument("--yfinal", type=float, default=None)
    ovr.add_argument("--zfinal", type=float, default=None)
    ovr.add_argument("--x-i", dest="x_i", type=float, default=None)
    ovr.add_argument("--y-i", dest="y_i", type=float, default=None)
    ovr.add_argument("--z-i", dest="z_i", type=float, default=None)
    ovr.add_argument("--multi-target", dest="multi_target", action="store_const", const=True,
                     default=None)
    ovr.add_argument("--cca-preset", dest="cca_preset", type=int, default=None,
                     choices=[1, 2, 3, 4, 5])

    # Runtime (non-Param) flags.
    rt = p.add_argument_group("runtime")
    rt.add_argument("--weather-path", type=Path, default=DEFAULT_WEATHER)
    rt.add_argument("--no-plot", dest="plot", action="store_false")
    rt.add_argument("--animate", action="store_true",
                    help="Show an interactive animated plot stepping through each rt frame")
    rt.add_argument("--save-video", type=Path, default=None)
    rt.add_argument("--dynamics", choices=["kinematic", "sixdof", "pybullet"], default="kinematic",
                    help="UAV dynamics model: kinematic (CCA3D), sixdof (L1 + PD), or pybullet")
    rt.add_argument("--drone-profile", dest="drone_profile", default="generic",
                    choices=available_profiles(),
                    help="Quadrotor profile for 6DoF dynamics (ignored in kinematic mode)")
    rt.add_argument("--drone-model", dest="drone_model", default="cf2x",
                    choices=["cf2x", "cf2p", "hb", "racer"],
                    help="PyBullet drone URDF model (only used with --dynamics pybullet)")
    rt.add_argument("--pybullet-gui", dest="pybullet_gui", action="store_true", default=True,
                    help="Open the PyBullet GUI window (default: True)")
    rt.add_argument("--no-pybullet-gui", dest="pybullet_gui", action="store_false",
                    help="Run PyBullet in headless (DIRECT) mode")
    rt.add_argument("--quiet", action="store_true")
    return p


def main(argv: list[str] | None = None) -> None:
    args = build_argparser().parse_args(argv)
    param = _build_param(args)
    if args.dump_config is not None:
        dump_config(param, args.dump_config)
        print(f"Resolved config written to {args.dump_config}")
        sys.exit(0)
    run(param, weather_path=args.weather_path, plot=args.plot,
        animate=args.animate, save_video=args.save_video,
        dynamics=args.dynamics, drone_profile=args.drone_profile,
        drone_model=args.drone_model, pybullet_gui=args.pybullet_gui)


if __name__ == "__main__":
    main()
