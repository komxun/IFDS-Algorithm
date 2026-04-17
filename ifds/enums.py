"""Enumerations shared across the IFDS package.

Lifted verbatim from the original ``ifds.config`` module so the values remain
identical to the MATLAB constants (``simMode``, ``useOptimizer``, ``scene``).
"""

from __future__ import annotations

from enum import IntEnum


class SimMode(IntEnum):
    """`simMode` in MATLAB."""

    BY_TIME = 1
    BY_DISTANCE = 2


class OptimizerMode(IntEnum):
    """`useOptimizer` in MATLAB."""

    OFF = 0
    GLOBAL = 1
    LOCAL = 2


class Scene(IntEnum):
    """Supported scenario identifiers (kept identical to the MATLAB `scene` values)."""

    CEILING = 0
    ONE_OBJECT = 1
    TWO_OBJECTS = 2
    THREE_OBJECTS = 3
    COMPLEX = 4
    MIXED_TRIO = 5
    NON_URBAN = 7
    URBAN = 12
    DYNAMIC_3 = 41
    DYNAMIC_4 = 42
    DYNAMIC_7 = 44
    FOUR_OBJ_69 = 69
    DYNAMIC_3_ALT = 6969
