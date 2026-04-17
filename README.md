IFDS-Algorithm — Dynamic UAV Autorouting (Python)
=================================================

Python port of Komsun Tamanakijprasart's MATLAB *Interfered Fluid Dynamical
System* (IFDS) path-planning project. Original video:
<https://youtu.be/XtmcNa-w4-0?si=V0FAj7HmrgcvlQuK>

Install
-------

```
python -m pip install -e .
```

Python 3.11+ required. Depends on `numpy`, `scipy`, `matplotlib`, `pyvista`,
`imageio-ffmpeg`.

Run
---

Pick a YAML preset from `configs/` (see [`configs/README.md`](configs/README.md)):

```
python -m scripts.run_main --config configs/scene3_static.yaml
python -m scripts.run_main --config configs/scene42_dynamic.yaml --rho0 0.5
python -m scripts.run_main --config configs/scene3_static.yaml --dump-config out.yaml
```

CLI-only also works (uses `Param()` defaults):

```
python -m scripts.run_main --scene 3 --env static         # default scenario
python -m scripts.run_main --scene 42 --env dynamic --k 1 # weather + moving obstacles
python -m scripts.run_main --scene 3 --optimizer 1        # global optimize rho0, sigma0
python -m scripts.run_main --no-plot --save-video out/run.mp4
```

Other entry points mirror the original MATLAB `main_*.m` scripts:

```
python -m scripts.run_alpha          # alpha sweep (main_ALPHA.m)
python -m scripts.run_sg             # multi-target demo (main_sg_generation.m)
python -m scripts.weather_gen --save # regenerate WeatherMat_<seed>
python -m scripts.realtime_analysis  # timing plots from data/real-time_analysis/
```

Tests
-----

```
pytest -q
```

Documentation
-------------

Developer design doc: [`docs/DESIGN.md`](docs/DESIGN.md).

Layout
------

- `ifds/`    – `enums.py`, `params.py`, `presets.py`, `config_io.py`, `scenes.py`,
  `objects.py`, `weather.py`, `optimizer.py`, `ifds.py`. `config.py` is a
  back-compat re-export shim.
- `uav/`     – CCA3D guidance and UAV dynamics (Kinematic3DoF today, SixDoF stub).
- `viz/`     – matplotlib 2D + PyVista 3D plotting and animation.
- `scripts/` – CLI entry points (ports of every MATLAB `main_*.m`).
- `configs/` – YAML scenario presets loaded via `--config`.
- `tests/`   – pytest regression suite.
- `data/`    – MATLAB `.mat` inputs (weather, reference trajectories).
- `legacy/`  – original `.m` files, preserved unchanged.

Original MATLAB README (historical notes below)
-----------------------------------------------

Final version for IRP Dynamic Autorouting\
Results Video: https://youtu.be/XtmcNa-w4-0?si=V0FAj7HmrgcvlQuK

# This version
- [x] Constraints matrix has been introduced (e.g. Weather data)
- [x] Path following and UAV dynamics have been introduced
- [x] Evaluation methods are considered (e.g. real-time performance)
- [x] Added Position-Holding feature
- [x] Added Global vs Local path adaptability based on scenarios  

# Not finished
- [ ] Fuel consumption and flight time cost not considered
- [ ] Overlapped shape problem not fixed
- [ ] Stagnation problem not fixed (e.g. when path is orthogonal to cylinder surface)


# Problem Noticed
- **Problem1**: The effect of overlapped object ruined the path planning result
- **Problem2**: The Barrier of the cylinder, cone, and parallel piped are not uniformly enclosed -> This is because of how the safeguard function is derived from sphere
- **Problem3**: After restructuring the code, the Global Optimizer ran a lot slower (but maybe more accurate than `verion2_legacy` since the objective function considers the SafeGuard and identical to the IFDS algorithm called in the main file)

# Possible Solutions
- **Problem1**: Follow the literature to solve the overlapped problem
- **Problem2**: Derive the barrier for the cylinder case, or even for the general case

# Results
![image](https://github.com/komxun/IFDS-Algorithm/assets/133139057/078c3a5d-717b-4cf6-a459-22dee9d5c450)




