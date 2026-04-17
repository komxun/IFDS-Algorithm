"""YAML configuration I/O for the ``Param`` dataclass.

Provides:

- :func:`load_config` — read a YAML file and return a fully-populated ``Param``.
- :func:`dump_config` — serialize a ``Param`` back to YAML (round-trip safe).
- :func:`apply_overrides` — merge CLI-style overrides onto an existing ``Param``.

The YAML schema is documented in ``configs/README.md``. Unknown keys raise
``ValueError`` so typos surface immediately.
"""

from __future__ import annotations

import dataclasses
from enum import IntEnum
from pathlib import Path
from typing import Any

import yaml

from ifds.enums import OptimizerMode, Scene, SimMode
from ifds.params import Param
from ifds.presets import CCATuning, cca_preset

# Known top-level YAML sections (keeps the schema flat and introspectable).
_TOP_LEVEL_KEYS = {"simulation", "scenario", "ifds", "weather", "uav", "cca"}

# Per-section allowed keys -> Param field name.
_SECTION_MAP: dict[str, dict[str, str]] = {
    "simulation": {
        "tsim": "tsim", "dt": "dt", "rtsim": "rtsim", "dt_traj": "dt_traj",
        "sim_mode": "sim_mode", "target_thresh": "target_thresh",
        "show_disp": "show_disp",
    },
    "scenario": {"scene": "scene", "multi_target": "multi_target"},
    "ifds": {
        "sf": "sf", "rho0_initial": "rho0_initial", "sigma0_initial": "sigma0_initial",
        "use_optimizer": "use_optimizer", "rg": "rg",
    },
    "weather": {
        "k": "k", "b_u": "b_u", "b_l": "b_l", "env": "env",
        "static_weather_index": "static_weather_index",
    },
    # `uav` uses nested structured values that get expanded below.
    "uav": {"c": "c"},
}

_ENUM_FIELDS: dict[str, type[IntEnum]] = {
    "sim_mode": SimMode,
    "scene": Scene,
    "use_optimizer": OptimizerMode,
}


def _to_enum(enum_cls: type[IntEnum], value: Any) -> IntEnum:
    """Coerce a YAML scalar to an ``IntEnum``, accepting int or name."""
    if isinstance(value, enum_cls):
        return value
    if isinstance(value, int):
        return enum_cls(value)
    if isinstance(value, str):
        try:
            return enum_cls[value]
        except KeyError as exc:
            valid = ", ".join(m.name for m in enum_cls)
            raise ValueError(
                f"Invalid {enum_cls.__name__} name {value!r}. Valid: {valid}"
            ) from exc
    raise ValueError(f"Cannot coerce {value!r} to {enum_cls.__name__}")


def _check_keys(section: str, data: dict[str, Any], allowed: set[str]) -> None:
    unknown = set(data) - allowed
    if unknown:
        raise ValueError(
            f"Unknown key(s) {sorted(unknown)} in section '{section}'. "
            f"Allowed: {sorted(allowed)}"
        )


def _parse_uav(section: dict[str, Any]) -> dict[str, Any]:
    """Expand the ``uav`` section into flat ``Param`` fields."""
    allowed = {"c", "start", "final", "pose"}
    _check_keys("uav", section, allowed)
    out: dict[str, Any] = {}
    if "c" in section:
        out["c"] = float(section["c"])
    if "start" in section:
        start = section["start"]
        if not isinstance(start, (list, tuple)) or len(start) != 3:
            raise ValueError("uav.start must be a 3-element list [x, y, z]")
        out["xini"], out["yini"], out["zini"] = map(float, start)
    if "final" in section:
        final = section["final"]
        if not isinstance(final, (list, tuple)) or len(final) != 3:
            raise ValueError("uav.final must be a 3-element list [x, y, z]")
        out["xfinal"], out["yfinal"], out["zfinal"] = map(float, final)
    if "pose" in section:
        pose = section["pose"]
        if not isinstance(pose, dict):
            raise ValueError("uav.pose must be a mapping {x, y, z, psi, gamma}")
        pose_allowed = {"x", "y", "z", "psi", "gamma"}
        _check_keys("uav.pose", pose, pose_allowed)
        mapping = {"x": "x_i", "y": "y_i", "z": "z_i", "psi": "psi_i", "gamma": "gamma_i"}
        for k, v in pose.items():
            out[mapping[k]] = float(v)
    return out


def _parse_cca(section: dict[str, Any]) -> CCATuning:
    """Handle either ``cca: {preset: N}`` or explicit ``{kappa, delta, kd}``."""
    allowed = {"preset", "kappa", "delta", "kd"}
    _check_keys("cca", section, allowed)
    has_preset = "preset" in section
    has_explicit = any(k in section for k in ("kappa", "delta", "kd"))
    if has_preset and has_explicit:
        raise ValueError("cca: use either 'preset' OR explicit {kappa, delta, kd}, not both")
    if has_preset:
        return cca_preset(int(section["preset"]))
    # Explicit path — missing values fall back to the default preset 5 values.
    default = cca_preset(5)
    return CCATuning(
        kappa=float(section.get("kappa", default.kappa)),
        delta=float(section.get("delta", default.delta)),
        kd=float(section.get("kd", default.kd)),
    )


def _parse_section(name: str, section: dict[str, Any]) -> dict[str, Any]:
    """Coerce a simple (non-structured) YAML section into Param-field kwargs."""
    allowed_keys = _SECTION_MAP[name]
    _check_keys(name, section, set(allowed_keys))
    out: dict[str, Any] = {}
    for yaml_key, value in section.items():
        field_name = allowed_keys[yaml_key]
        if field_name in _ENUM_FIELDS:
            value = _to_enum(_ENUM_FIELDS[field_name], value)
        out[field_name] = value
    return out


def load_config(path: str | Path) -> Param:
    """Load a YAML config file and return a fully-populated :class:`Param`.

    Missing sections fall back to :class:`Param` dataclass defaults.
    """
    path = Path(path)
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Top-level YAML must be a mapping; got {type(data).__name__}")
    unknown = set(data) - _TOP_LEVEL_KEYS
    if unknown:
        raise ValueError(
            f"Unknown top-level section(s) {sorted(unknown)}. "
            f"Allowed: {sorted(_TOP_LEVEL_KEYS)}"
        )

    kwargs: dict[str, Any] = {}
    for name in ("simulation", "scenario", "ifds", "weather"):
        if name in data:
            kwargs.update(_parse_section(name, data[name] or {}))
    if "uav" in data:
        kwargs.update(_parse_uav(data["uav"] or {}))
    if "cca" in data:
        kwargs["cca"] = _parse_cca(data["cca"] or {})
    return Param(**kwargs)


def dump_config(param: Param, path: str | Path) -> None:
    """Serialize ``param`` to a YAML file, keeping the structured layout."""
    doc = {
        "simulation": {
            "tsim": int(param.tsim),
            "dt": float(param.dt),
            "rtsim": int(param.rtsim),
            "dt_traj": float(param.dt_traj),
            "sim_mode": param.sim_mode.name,
            "target_thresh": float(param.target_thresh),
            "show_disp": bool(param.show_disp),
        },
        "scenario": {
            "scene": param.scene.name,
            "multi_target": bool(param.multi_target),
        },
        "ifds": {
            "sf": bool(param.sf),
            "rho0_initial": float(param.rho0_initial),
            "sigma0_initial": float(param.sigma0_initial),
            "use_optimizer": param.use_optimizer.name,
            "rg": float(param.rg),
        },
        "weather": {
            "k": float(param.k),
            "b_u": float(param.b_u),
            "b_l": float(param.b_l),
            "env": str(param.env),
            "static_weather_index": int(param.static_weather_index),
        },
        "uav": {
            "c": float(param.c),
            "start": [float(param.xini), float(param.yini), float(param.zini)],
            "final": [float(param.xfinal), float(param.yfinal), float(param.zfinal)],
            "pose": {
                "x": float(param.x_i), "y": float(param.y_i), "z": float(param.z_i),
                "psi": float(param.psi_i), "gamma": float(param.gamma_i),
            },
        },
        "cca": {
            "kappa": float(param.cca.kappa),
            "delta": float(param.cca.delta),
            "kd": float(param.cca.kd),
        },
    }
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, sort_keys=False, default_flow_style=False)


# Flat override keys accepted by :func:`apply_overrides` / CLI flags.
_OVERRIDE_FIELDS = {f.name for f in dataclasses.fields(Param)} | {"cca_preset"}


def apply_overrides(param: Param, **kwargs: Any) -> Param:
    """Return a new ``Param`` with ``kwargs`` merged on top.

    ``kwargs`` keys must be either ``Param`` field names or ``"cca_preset"``
    (shortcut for ``cca=cca_preset(N)``). ``None`` values are ignored so the
    caller can pass every CLI flag regardless of whether the user set it.
    """
    unknown = set(kwargs) - _OVERRIDE_FIELDS
    if unknown:
        raise ValueError(f"Unknown override field(s): {sorted(unknown)}")

    effective = {k: v for k, v in kwargs.items() if v is not None}
    if "cca_preset" in effective:
        effective["cca"] = cca_preset(int(effective.pop("cca_preset")))
    for field_name, enum_cls in _ENUM_FIELDS.items():
        if field_name in effective:
            effective[field_name] = _to_enum(enum_cls, effective[field_name])
    return dataclasses.replace(param, **effective)
