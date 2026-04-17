"""Back-compat re-exports.

Prefer importing from the dedicated modules:

- :mod:`ifds.enums`   — ``SimMode``, ``OptimizerMode``, ``Scene``
- :mod:`ifds.params`  — ``Param``
- :mod:`ifds.presets` — ``CCATuning``, ``cca_preset``, ``num_objects_for_scene``
- :mod:`ifds.config_io` — YAML load/dump helpers

This module exists so existing ``from ifds.config import ...`` call sites keep
working after the schema split.
"""

from __future__ import annotations

from ifds.enums import OptimizerMode, Scene, SimMode
from ifds.params import Param
from ifds.presets import CCATuning, cca_preset, num_objects_for_scene

__all__ = [
    "CCATuning",
    "OptimizerMode",
    "Param",
    "Scene",
    "SimMode",
    "cca_preset",
    "num_objects_for_scene",
]
