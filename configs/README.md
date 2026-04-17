# Configs

YAML scenarios consumed by `scripts/run_main.py --config PATH`. Unknown keys
raise; missing sections fall back to the `Param` dataclass defaults in
`ifds/params.py`.

## Shipped presets

| File | Scene | Env | Notes |
| --- | --- | --- | --- |
| `default.yaml` | THREE_OBJECTS | static | Mirrors `Param()` defaults. |
| `scene3_static.yaml` | THREE_OBJECTS | static | Main quick-start demo. |
| `scene42_dynamic.yaml` | DYNAMIC_4 | dynamic (k=1) | 4 moving obstacles + weather. |
| `scene2_multi_sg.yaml` | TWO_OBJECTS | static | Multi-target streamline demo. |
| `scene1_alpha.yaml` | ONE_OBJECT | static | Base for alpha-angle sweeps. |
| `weather_test.yaml` | CEILING | dynamic (k=1) | Minimal weather sanity. |

## Schema

Top-level sections: `simulation`, `scenario`, `ifds`, `weather`, `uav`, `cca`.
All are optional.

```yaml
simulation:
  tsim: int                # waypoint-buffer length (BY_TIME also uses as #steps)
  dt: float                # IFDS integration step [s]
  rtsim: int               # number of outer (replan) iterations
  dt_traj: float           # CCA path-follow duration per rt [s]
  sim_mode: BY_TIME | BY_DISTANCE | 1 | 2
  target_thresh: float     # distance-to-target stop threshold [m]
  show_disp: bool

scenario:
  scene: THREE_OBJECTS | 3 | ...  # any Scene enum name or int
  multi_target: bool

ifds:
  sf: bool                  # shape-following mode
  rho0_initial: float
  sigma0_initial: float
  use_optimizer: OFF | GLOBAL | LOCAL | 0 | 1 | 2
  rg: float                 # allowed gap (delta_g)

weather:
  k: float                  # weather effect strength (0 disables)
  b_u: float                # upper bound
  b_l: float                # lower bound
  env: static | dynamic
  static_weather_index: int # frame picked under env=static

uav:
  c: float                  # speed [m/s]
  start: [x, y, z]          # target seed
  final: [x, y, z]          # destination
  pose:                     # UAV initial pose
    x: float
    y: float
    z: float
    psi: float
    gamma: float

cca:
  # EITHER
  preset: 1..5              # loads _CCA_PRESETS[N]
  # OR
  kappa: float
  delta: float
  kd: float
```

## CLI overrides

Any flag on `scripts/run_main.py` overrides the YAML value:

```
python -m scripts.run_main --config configs/scene3_static.yaml --rho0 0.5 --rtsim 3
```

## Reproducibility

Dump the fully resolved `Param` back to YAML:

```
python -m scripts.run_main --config configs/scene3_static.yaml --rho0 0.5 --dump-config out.yaml
```
