"""Tests for ``ifds.config_io`` YAML loader, dumper, and override merger."""

from __future__ import annotations

from pathlib import Path

import pytest

from ifds.config_io import apply_overrides, dump_config, load_config
from ifds.enums import OptimizerMode, Scene, SimMode
from ifds.params import Param
from ifds.presets import cca_preset

REPO_ROOT = Path(__file__).resolve().parents[1]
CONFIGS = REPO_ROOT / "configs"


# --- load_config --------------------------------------------------------------


def test_load_default_matches_param_defaults() -> None:
    """`configs/default.yaml` is meant to mirror `Param()` defaults exactly."""
    loaded = load_config(CONFIGS / "default.yaml")
    expected = Param()
    assert loaded == expected


def test_load_scene3_static() -> None:
    p = load_config(CONFIGS / "scene3_static.yaml")
    assert p.scene == Scene.THREE_OBJECTS
    assert p.sim_mode == SimMode.BY_DISTANCE
    assert p.use_optimizer == OptimizerMode.OFF
    assert p.env == "static"
    assert p.cca == cca_preset(5)
    assert p.xini == 0.0 and p.yfinal == 0.0


def test_load_scene42_dynamic() -> None:
    p = load_config(CONFIGS / "scene42_dynamic.yaml")
    assert p.scene == Scene.DYNAMIC_4
    assert p.env == "dynamic"
    assert p.k == 1.0
    assert p.b_u == 0.7


def test_enum_accepts_int(tmp_path: Path) -> None:
    cfg = tmp_path / "c.yaml"
    cfg.write_text("simulation:\n  sim_mode: 2\nscenario:\n  scene: 3\n")
    p = load_config(cfg)
    assert p.sim_mode == SimMode.BY_DISTANCE
    assert p.scene == Scene.THREE_OBJECTS


def test_enum_accepts_name(tmp_path: Path) -> None:
    cfg = tmp_path / "c.yaml"
    cfg.write_text(
        "simulation:\n  sim_mode: BY_DISTANCE\n"
        "scenario:\n  scene: THREE_OBJECTS\n"
        "ifds:\n  use_optimizer: LOCAL\n"
    )
    p = load_config(cfg)
    assert p.sim_mode == SimMode.BY_DISTANCE
    assert p.scene == Scene.THREE_OBJECTS
    assert p.use_optimizer == OptimizerMode.LOCAL


def test_cca_explicit(tmp_path: Path) -> None:
    cfg = tmp_path / "c.yaml"
    cfg.write_text("cca:\n  kappa: 7.5\n  delta: 3.0\n  kd: 0.25\n")
    p = load_config(cfg)
    assert p.cca.kappa == 7.5
    assert p.cca.delta == 3.0
    assert p.cca.kd == 0.25


def test_cca_preset_only(tmp_path: Path) -> None:
    cfg = tmp_path / "c.yaml"
    cfg.write_text("cca:\n  preset: 3\n")
    p = load_config(cfg)
    assert p.cca == cca_preset(3)


def test_cca_conflict_raises(tmp_path: Path) -> None:
    cfg = tmp_path / "c.yaml"
    cfg.write_text("cca:\n  preset: 3\n  kappa: 7.5\n")
    with pytest.raises(ValueError, match="either 'preset' OR explicit"):
        load_config(cfg)


def test_unknown_top_level_raises(tmp_path: Path) -> None:
    cfg = tmp_path / "c.yaml"
    cfg.write_text("bogus_section:\n  x: 1\n")
    with pytest.raises(ValueError, match="Unknown top-level section"):
        load_config(cfg)


def test_unknown_nested_key_raises(tmp_path: Path) -> None:
    cfg = tmp_path / "c.yaml"
    cfg.write_text("simulation:\n  bogus: 123\n")
    with pytest.raises(ValueError, match="Unknown key"):
        load_config(cfg)


def test_invalid_enum_name_raises(tmp_path: Path) -> None:
    cfg = tmp_path / "c.yaml"
    cfg.write_text("scenario:\n  scene: NOT_A_SCENE\n")
    with pytest.raises(ValueError, match="Invalid Scene name"):
        load_config(cfg)


# --- dump_config round-trip ---------------------------------------------------


@pytest.mark.parametrize(
    "fname",
    [
        "default.yaml",
        "scene3_static.yaml",
        "scene42_dynamic.yaml",
        "scene2_multi_sg.yaml",
        "scene1_alpha.yaml",
        "weather_test.yaml",
    ],
)
def test_roundtrip(fname: str, tmp_path: Path) -> None:
    src = load_config(CONFIGS / fname)
    out = tmp_path / fname
    dump_config(src, out)
    reloaded = load_config(out)
    assert reloaded == src


# --- apply_overrides ----------------------------------------------------------


def test_apply_overrides_ignores_none() -> None:
    p = Param()
    p2 = apply_overrides(p, rho0_initial=None, tsim=None)
    assert p2 == p


def test_apply_overrides_replaces_values() -> None:
    p = Param()
    p2 = apply_overrides(p, rho0_initial=0.25, rtsim=7)
    assert p2.rho0_initial == 0.25
    assert p2.rtsim == 7
    # Unchanged field preserved:
    assert p2.tsim == p.tsim


def test_apply_overrides_coerces_enums() -> None:
    p = Param()
    p2 = apply_overrides(p, scene=42, sim_mode="BY_TIME", use_optimizer=2)
    assert p2.scene == Scene.DYNAMIC_4
    assert p2.sim_mode == SimMode.BY_TIME
    assert p2.use_optimizer == OptimizerMode.LOCAL


def test_apply_overrides_cca_preset_shortcut() -> None:
    p = Param()
    p2 = apply_overrides(p, cca_preset=2)
    assert p2.cca == cca_preset(2)


def test_apply_overrides_unknown_field_raises() -> None:
    p = Param()
    with pytest.raises(ValueError, match="Unknown override field"):
        apply_overrides(p, nonexistent=123)


# --- CLI integration ----------------------------------------------------------


def test_cli_build_param_uses_config() -> None:
    from scripts.run_main import _build_param, build_argparser

    parser = build_argparser()
    args = parser.parse_args(["--config", str(CONFIGS / "scene3_static.yaml")])
    param = _build_param(args)
    assert param.scene == Scene.THREE_OBJECTS
    assert param.sim_mode == SimMode.BY_DISTANCE


def test_cli_overrides_beat_config() -> None:
    from scripts.run_main import _build_param, build_argparser

    parser = build_argparser()
    args = parser.parse_args([
        "--config", str(CONFIGS / "scene3_static.yaml"),
        "--rho0", "0.25", "--rtsim", "7",
    ])
    param = _build_param(args)
    assert param.rho0_initial == 0.25
    assert param.rtsim == 7
    # Field not overridden stays from YAML:
    assert param.scene == Scene.THREE_OBJECTS


def test_cli_without_config_uses_defaults() -> None:
    from scripts.run_main import _build_param, build_argparser

    parser = build_argparser()
    args = parser.parse_args([])
    param = _build_param(args)
    assert param == Param()
