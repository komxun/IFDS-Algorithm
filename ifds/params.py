"""`Param` dataclass — every scalar knob for an IFDS + CCA + weather simulation.

Field names mirror the MATLAB ``Param`` table in ``main.m``.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from ifds.enums import OptimizerMode, Scene, SimMode
from ifds.presets import CCATuning, cca_preset, num_objects_for_scene


@dataclass
class Param:
    """All scalar knobs for the IFDS + CCA + weather simulation."""

    # Simulation
    tsim: int = 100
    dt: float = 0.1
    rtsim: int = 50
    dt_traj: float = 1.0
    sim_mode: SimMode = SimMode.BY_DISTANCE
    target_thresh: float = 2.0
    show_disp: bool = True

    # Scenario
    scene: Scene = Scene.THREE_OBJECTS
    num_obj: int = 3
    multi_target: bool = False

    # IFDS tuning
    sf: bool = False
    rho0_initial: float = 1.0
    sigma0_initial: float = 1.0
    use_optimizer: OptimizerMode = OptimizerMode.OFF
    rg: float = 10.0  # minimum allowed gap distance (delta_g)

    # Weather constraint matrix
    k: float = 0.0
    b_u: float = 0.9
    b_l: float = 0.0

    # UAV speed + start/target
    c: float = 10.0
    xini: float = 0.0
    yini: float = 0.0
    zini: float = 0.0
    xfinal: float = 200.0
    yfinal: float = 0.0
    zfinal: float = 0.0

    # UAV initial pose
    x_i: float = 0.0
    y_i: float = -20.0
    z_i: float = 5.0
    psi_i: float = 0.0
    gamma_i: float = 0.0

    # CCA guidance
    cca: CCATuning = field(default_factory=lambda: cca_preset(5))

    # Environment mode
    env: str = "static"  # "static" | "dynamic"

    # Weather frame used under "static" env (15 matches main.m default)
    static_weather_index: int = 15

    def __post_init__(self) -> None:
        self.scene = Scene(int(self.scene))
        self.sim_mode = SimMode(int(self.sim_mode))
        self.use_optimizer = OptimizerMode(int(self.use_optimizer))
        self.num_obj = num_objects_for_scene(self.scene)
