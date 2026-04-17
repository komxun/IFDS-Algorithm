"""Fixed lookup tables: scene -> object count, CCA preset id -> tuning.

These constants are not user-tunable via the YAML layer; they are part of the
algorithmic definition of the scenes / CCA presets.
"""

from __future__ import annotations

from dataclasses import dataclass

from ifds.enums import Scene


@dataclass
class CCATuning:
    """Carrot-Chasing-Algorithm guidance gains (`kappa`, `delta`, `kd`)."""

    kappa: float = 50.0
    delta: float = 20.0
    kd: float = 0.0


_SCENE_NUMOBJ: dict[Scene, int] = {
    Scene.CEILING: 1,
    Scene.ONE_OBJECT: 1,
    Scene.TWO_OBJECTS: 2,
    Scene.THREE_OBJECTS: 3,
    Scene.COMPLEX: 3,
    Scene.MIXED_TRIO: 3,
    Scene.NON_URBAN: 7,
    Scene.URBAN: 12,
    Scene.DYNAMIC_3: 3,
    Scene.DYNAMIC_4: 4,
    Scene.DYNAMIC_7: 7,
    Scene.FOUR_OBJ_69: 4,
    Scene.DYNAMIC_3_ALT: 3,
}


def num_objects_for_scene(scene: Scene | int) -> int:
    """Return the object count for a scenario id, matching the MATLAB switch blocks."""
    return _SCENE_NUMOBJ[Scene(int(scene))]


_CCA_PRESETS: dict[int, CCATuning] = {
    1: CCATuning(kappa=10, delta=2, kd=0),
    2: CCATuning(kappa=20, delta=5, kd=0),
    3: CCATuning(kappa=10, delta=10, kd=0.1),
    4: CCATuning(kappa=100, delta=1, kd=0),
    5: CCATuning(kappa=50, delta=20, kd=0),
}


def cca_preset(preset: int) -> CCATuning:
    """Retrieve a CCA tuning preset (1..5) as used in `main.m`."""
    return _CCA_PRESETS[preset]
