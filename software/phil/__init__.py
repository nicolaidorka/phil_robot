"""Phil robot control package.

High-level, calibrated, smooth XYZ + go-to-well control for the Phil arm
robot (which runs on the Squid microcontroller firmware).

    from phil import PhilRobot, WellPlate, Calibration
"""
from .geometry.well_plate import WellPlate, Well
from .geometry.calibration import Calibration, ReferencePoint
from .teaching.teach import TeachTable
from .robot import PhilRobot, SimulatedBackend, PhilHandshakeError
from . import constants

__all__ = [
    "PhilRobot",
    "SimulatedBackend",
    "PhilHandshakeError",
    "WellPlate",
    "Well",
    "Calibration",
    "ReferencePoint",
    "TeachTable",
    "constants",
]
