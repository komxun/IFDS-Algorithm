"""Sanity checks for obstacle shape factories and scene dispatcher."""

from __future__ import annotations

import numpy as np
import pytest

from ifds.config import Scene, num_objects_for_scene
from ifds.objects import Object
from ifds.scenes import (
    allocate_objects,
    build_scene,
    create_cone,
    create_cylinder,
    create_pipe,
    create_sphere,
)


@pytest.mark.parametrize("factory,args", [
    (create_sphere, (100, 0, 0, 50)),
    (create_cylinder, (100, 0, 0, 30, 50)),
    (create_cone, (100, 0, 0, 30, 50)),
    (create_pipe, (100, 0, 0, 30, 50)),
])
def test_surface_point_has_gamma_one(factory, args):
    """A point on the +x surface axis should satisfy Gamma ≈ 1."""
    obj = Object()
    probe_x = args[0] + args[3] / 2  # x0 + a
    factory(obj, probe_x, 0, args[2], *args)
    assert np.isclose(obj.gamma, 1.0, atol=1e-6)


def test_origin_has_gamma_zero_ish():
    """At the exact center, Gamma must be 0 (all terms vanish)."""
    obj = Object()
    create_sphere(obj, 100.0, 0.0, 0.0, 100.0, 0.0, 0.0, 50.0)
    assert obj.gamma == pytest.approx(0.0)


def test_allocate_objects_count():
    for s in [Scene.ONE_OBJECT, Scene.TWO_OBJECTS, Scene.THREE_OBJECTS,
              Scene.URBAN, Scene.NON_URBAN, Scene.DYNAMIC_4]:
        assert len(allocate_objects(s)) == num_objects_for_scene(s)


def test_build_scene_populates_gamma_and_normal():
    objs = allocate_objects(Scene.THREE_OBJECTS)
    build_scene(Scene.THREE_OBJECTS, objs, 0.0, 0.0, 0.0, rt=0)
    for o in objs:
        assert o.gamma > 0.0
        assert o.n.shape == (3,)
        assert o.t.shape == (3,)
        assert np.linalg.norm(o.n) > 0.0
