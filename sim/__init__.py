"""PyBullet simulation package for IFDS dynamic autorouting.

Provides a simplified PyBullet drone environment, PID controllers migrated
from ``gym-pybullet-drones-routing``, weather ground-plane texture rendering,
and an integration runner that drives a drone through IFDS waypoints.
"""

from sim.aviary import Aviary
from sim.control import DSLPIDControl, PIDVelocityControl
from sim.enums import DroneModel, Physics
from sim.runner import run_pybullet
from sim.weather_texture import WeatherGroundPlane

__all__ = [
    "Aviary",
    "DSLPIDControl",
    "PIDVelocityControl",
    "DroneModel",
    "Physics",
    "WeatherGroundPlane",
    "run_pybullet",
]
