"""Geometry: labware coordinates and the mm <-> joint position models.

Only the always-available (numpy-only) pieces are re-exported here.
``kinematics`` and ``well_map`` need scipy (optional) and are imported directly
by ``phil.robot`` under try/except, so importing this subpackage never requires scipy.
"""
from .well_plate import WellPlate, Well
from .calibration import Calibration, ReferencePoint

__all__ = ["WellPlate", "Well", "Calibration", "ReferencePoint"]
