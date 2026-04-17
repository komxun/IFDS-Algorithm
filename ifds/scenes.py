"""Obstacle shape factories and scene dispatcher.

Ports the MATLAB ``create_scene`` switch block and its nested
``create_sphere``/``create_cylinder``/``create_cone``/``create_pipe``/``create_ceiling``
helpers from ``IFDS.m``.
"""

from __future__ import annotations

import math
from collections.abc import Callable

import numpy as np

from ifds.enums import Scene
from ifds.objects import Object
from ifds.presets import num_objects_for_scene


def _apply_shape(
    obj: Object,
    x: float,
    y: float,
    z: float,
    x0: float,
    y0: float,
    z0: float,
    a: float,
    b: float,
    c: float,
    p: float,
    q: float,
    r: float,
    alpha_deg: float,
    rotated_tangent: bool,
) -> None:
    """Populate ``obj`` with geometry + current Gamma/n/t at the UAV location."""
    gamma = ((x - x0) / a) ** (2 * p) + ((y - y0) / b) ** (2 * q) + ((z - z0) / c) ** (2 * r)
    dgdx = (2 * p * ((x - x0) / a) ** (2 * p - 1)) / a
    dgdy = (2 * q * ((y - y0) / b) ** (2 * q - 1)) / b
    dgdz = (2 * r * ((z - z0) / c) ** (2 * r - 1)) / c
    n = np.array([dgdx, dgdy, dgdz], dtype=float)

    if rotated_tangent:
        alpha = math.radians(alpha_deg)
        rot = np.array(
            [
                [dgdy, dgdx * dgdz, dgdx],
                [-dgdx, dgdy * dgdz, dgdy],
                [0.0, -(dgdx**2) - (dgdy**2), dgdz],
            ],
            dtype=float,
        )
        t = rot @ np.array([math.cos(alpha), math.sin(alpha), 0.0])
    else:
        t = np.array([dgdy, -dgdx, 0.0], dtype=float)

    obj.origin = np.array([x0, y0, z0], dtype=float)
    obj.a, obj.b, obj.c = a, b, c
    obj.p, obj.q, obj.r = p, q, r
    obj.rstar = min(a, b, c)
    obj.alpha_deg = alpha_deg
    obj.gamma = gamma
    obj.n = n
    obj.t = t


def create_sphere(obj: Object, x: float, y: float, z: float, x0: float, y0: float, z0: float,
                  diameter: float, alpha_deg: float = 0.0) -> None:
    a = b = c = diameter / 2
    _apply_shape(obj, x, y, z, x0, y0, z0, a, b, c, p=1, q=1, r=1,
                 alpha_deg=alpha_deg, rotated_tangent=True)


def create_cylinder(obj: Object, x: float, y: float, z: float, x0: float, y0: float, z0: float,
                    diameter: float, height: float, alpha_deg: float = 0.0) -> None:
    a = b = diameter / 2
    c = height
    _apply_shape(obj, x, y, z, x0, y0, z0, a, b, c, p=1, q=1, r=4,
                 alpha_deg=alpha_deg, rotated_tangent=True)


def create_cone(obj: Object, x: float, y: float, z: float, x0: float, y0: float, z0: float,
                diameter: float, height: float) -> None:
    a = b = diameter / 2
    c = height
    _apply_shape(obj, x, y, z, x0, y0, z0, a, b, c, p=1, q=1, r=0.5,
                 alpha_deg=0.0, rotated_tangent=False)


def create_pipe(obj: Object, x: float, y: float, z: float, x0: float, y0: float, z0: float,
                diameter: float, height: float) -> None:
    a = b = diameter / 2
    c = height
    _apply_shape(obj, x, y, z, x0, y0, z0, a, b, c, p=2, q=2, r=2,
                 alpha_deg=0.0, rotated_tangent=False)


def create_ceiling(obj: Object, x: float, y: float, z: float, x0: float, y0: float, z0: float,
                   width: float, thickness: float) -> None:
    z0_shifted = z0 + thickness
    a = b = width / 2
    c = thickness
    _apply_shape(obj, x, y, z, x0, y0, z0_shifted, a, b, c, p=20, q=20, r=20,
                 alpha_deg=0.0, rotated_tangent=False)


SceneBuilder = Callable[[list[Object], float, float, float, int, float], None]


def _scene0(obj, x, y, z, rt, alpha):
    create_ceiling(obj[0], x, y, z, 100, 0, 50, 200, 10)


def _scene1(obj, x, y, z, rt, alpha):
    create_sphere(obj[0], x, y, z, 100, 0, 50, 50, alpha)


def _scene2(obj, x, y, z, rt, alpha):
    create_cylinder(obj[0], x, y, z, 60, 5, 0, 30, 50, alpha)
    create_sphere(obj[1], x, y, z, 120, -10, 0, 50, alpha)


def _scene3(obj, x, y, z, rt, alpha):
    create_cylinder(obj[0], x, y, z, 60, 5, 0, 30, 50, alpha)
    create_sphere(obj[1], x, y, z, 120, -10, 0, 50, alpha)
    create_cone(obj[2], x, y, z, 168, 0, 0, 25, 80)


def _scene4(obj, x, y, z, rt, alpha):
    create_cylinder(obj[0], x, y, z, 100, 5, 0, 25, 200, alpha)
    create_pipe(obj[1], x, y, z, 60, 20, 60, 80, 5)
    create_pipe(obj[2], x, y, z, 130, -30, 30, 100, 50)


def _scene5(obj, x, y, z, rt, alpha):
    create_cylinder(obj[0], x, y, z, 50, -20, 0, 30, 50, alpha)
    create_cone(obj[1], x, y, z, 100, -20, 0, 30, 50)
    create_pipe(obj[2], x, y, z, 150, -20, 0, 30, 50)


def _scene7(obj, x, y, z, rt, alpha):
    create_cone(obj[0], x, y, z, 60, 8, 0, 70, 50)
    create_cone(obj[1], x, y, z, 100, -24, 0, 89, 100)
    create_cone(obj[2], x, y, z, 160, 40, -4, 100, 30)
    create_cone(obj[3], x, y, z, 100, 100, -10, 150, 100)
    create_cone(obj[4], x, y, z, 180, -70, -10, 150, 20)
    create_cone(obj[5], x, y, z, 75, -75, -10, 150, 40)
    create_cylinder(obj[6], x, y, z, 170, -6, 0, 34, 100, alpha)


def _scene12(obj, x, y, z, rt, alpha):
    create_cylinder(obj[0], x, y, z, 100, 5, 0, 30, 50, alpha)
    create_pipe(obj[1], x, y, z, 140, 20, 0, 40, 10)
    create_pipe(obj[2], x, y, z, 20, 20, 0, 24, 40)
    create_pipe(obj[3], x, y, z, 55, -20, 0, 28, 50)
    create_sphere(obj[4], x, y, z, 53, -60, 0, 50, alpha)
    create_pipe(obj[5], x, y, z, 150, -80, 0, 40, 50)
    create_cone(obj[6], x, y, z, 100, -35, 0, 50, 45)
    create_cone(obj[7], x, y, z, 170, 2, 0, 20, 50)
    create_cone(obj[8], x, y, z, 60, 35, 0, 50, 30)
    create_cylinder(obj[9], x, y, z, 110, 70, 0, 60, 50, alpha)
    create_pipe(obj[10], x, y, z, 170, 60, 0, 40, 27)
    create_cone(obj[11], x, y, z, 150, -30, 0, 32, 45)


def _scene41(obj, x, y, z, rt, alpha):
    create_cylinder(obj[0], x, y, z, 100 + 50 * math.sin(rt / 8), 0 + 50 * math.cos(rt / 8), 0, 20, 80, alpha)
    create_sphere(obj[1], x, y, z, 100, 0, 0, 30, alpha)
    create_cylinder(obj[2], x, y, z, 100 - 50 * math.sin(rt / 8), 0 - 50 * math.cos(rt / 8), 0, 20, 50, alpha)


def _scene42(obj, x, y, z, rt, alpha):
    oy1 = -5 + 60 * math.cos(0.4 * rt)
    oy2 = -20 - 20 * math.sin(0.8 * rt)
    oz2 = 60 + 20 * math.cos(0.8 * rt)
    create_cylinder(obj[0], x, y, z, 40, 5, 0, 30, 40, alpha)
    create_cone(obj[1], x, y, z, 120, -10, 0, 25, 80)
    create_cylinder(obj[2], x, y, z, 80, oy1, 0, 10, 60, alpha)
    create_sphere(obj[3], x, y, z, 160, oy2, oz2, 20, alpha)


def _scene44(obj, x, y, z, rt, alpha):
    oy1 = 0 + 60 * math.sin(0.7 * rt)
    oy2 = 0 + 60 * math.cos(0.7 * rt)
    shift = 40 * math.sin(0.5 * rt)
    create_cylinder(obj[0], x, y, z, 40, 5, 0, 30, 80, alpha)
    create_pipe(obj[1], x, y, z, 40, -50, 0, 50, 30)
    create_pipe(obj[2], x, y, z, 150, 50, 0, 40, 60)
    create_pipe(obj[3], x, y, z, 150, -10, 0, 40, 80)
    create_pipe(obj[4], x, y, z, 110, oy1, 0, 20, 50)
    create_pipe(obj[5], x, y, z, 80, oy2, 0, 30, 30)
    create_sphere(obj[6], x, y, z, 100 + shift, 0 + shift, 60, 30, alpha)


def _scene69(obj, x, y, z, rt, alpha):
    create_cylinder(obj[0], x, y, z, 100, 5, 0, 30, 80, alpha)
    create_sphere(obj[1], x, y, z, 100, 30, 0, 40, alpha)
    create_sphere(obj[2], x, y, z, 100, -20, 0, 40, alpha)
    create_sphere(obj[3], x, y, z, 100, 5, 80, 30, alpha)


def _scene6969(obj, x, y, z, rt, alpha):
    create_sphere(obj[0], x, y, z, 100 + 30 * math.sin(rt / 8), 0 + 30 * math.cos(rt / 8), 0, 40, alpha)
    create_cylinder(obj[1], x, y, z, 100, 0, 0, 40, 80, alpha)
    create_sphere(obj[2], x, y, z, 100 - 30 * math.sin(rt / 8), 0 - 30 * math.cos(rt / 8), 0, 40, alpha)


_SCENE_BUILDERS: dict[Scene, SceneBuilder] = {
    Scene.CEILING: _scene0,
    Scene.ONE_OBJECT: _scene1,
    Scene.TWO_OBJECTS: _scene2,
    Scene.THREE_OBJECTS: _scene3,
    Scene.COMPLEX: _scene4,
    Scene.MIXED_TRIO: _scene5,
    Scene.NON_URBAN: _scene7,
    Scene.URBAN: _scene12,
    Scene.DYNAMIC_3: _scene41,
    Scene.DYNAMIC_4: _scene42,
    Scene.DYNAMIC_7: _scene44,
    Scene.FOUR_OBJ_69: _scene69,
    Scene.DYNAMIC_3_ALT: _scene6969,
}


def build_scene(
    scene: Scene | int,
    objects: list[Object],
    x: float,
    y: float,
    z: float,
    rt: int,
    alpha_deg: float = 0.0,
) -> list[Object]:
    """Populate ``objects`` in-place for the given ``scene`` at UAV point (x,y,z).

    Returns the same list for chaining. Matches ``create_scene`` in ``IFDS.m``.
    """
    builder = _SCENE_BUILDERS[Scene(int(scene))]
    builder(objects, x, y, z, rt, alpha_deg)
    return objects


def allocate_objects(scene: Scene | int) -> list[Object]:
    """Allocate a fresh ``list[Object]`` of the correct length for a scene."""
    return [Object() for _ in range(num_objects_for_scene(scene))]
