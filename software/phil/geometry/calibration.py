"""Calibration: map plate-local well coordinates -> physical robot XYZ.

The plate sits somewhere on the Phil robot at some position and (possibly)
a slight rotation/tilt.  Calibration captures that with a small set of
*reference points*: for a few known wells you jog the arm to the real
well center and record the robot (X, Y, Z).  From those we fit a linear
model

    X = ax*px + bx*py + cx
    Y = ay*px + by*py + cy
    Z = az*px + bz*py + cz

where (px, py) is the well's plate-local coordinate.  This single model
absorbs translation, in-plane rotation, independent X/Y scale, skew, and
plate tilt (the Z-plane), so every one of the 96 wells is reachable from
just a handful of measured points.

Recommended reference wells: three corners, e.g. ``A1``, ``A12``, ``H1``
(or add ``H12`` for a least-squares over-fit).  Fewer points still work:

    * 1 point  -> translation only (axes assumed aligned to the robot;
                  flip with ``axis_sign_x`` / ``axis_sign_y`` if needed).
    * 2 points -> in-plane rotation + uniform scale + translation;
                  Z held constant at the mean of the two.
    * >=3 pts  -> full affine + tilted Z-plane via least squares.

Per-well overrides (``dx``/``dy``/``dz`` in robot mm) are added on top of
the fitted model, so any individual well can be nudged.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field

import numpy as np

from .. import constants as C
from .well_plate import WellPlate
from ..paths import DEFAULT_CALIBRATION


@dataclass
class ReferencePoint:
    well: str
    robot: tuple[float, float, float]   # measured robot (X, Y, Z) in mm


@dataclass
class Calibration:
    """Plate-local -> robot transform plus travel heights and overrides."""

    labware: str = "corning_96_wellplate_360ul_flat.json"
    reference_points: list[ReferencePoint] = field(default_factory=list)
    well_overrides: dict[str, dict] = field(default_factory=dict)

    # travel / safety heights in robot Z mm
    z_safe_mm: float = 0.0          # height to retract to before XY travel
    z_clearance_mm: float = 5.0     # extra lift above the working plane during travel

    # used only when there is a single reference point (axes assumed aligned)
    axis_sign_x: float = 1.0
    axis_sign_y: float = 1.0

    # fitted model: each is (a, b, c) so value = a*px + b*py + c. Filled by fit().
    _model_x: tuple[float, float, float] | None = None
    _model_y: tuple[float, float, float] | None = None
    _model_z: tuple[float, float, float] | None = None
    _rms_error_mm: float | None = None

    # ------------------------------------------------------ nominal default
    @classmethod
    def nominal(cls, plate: WellPlate) -> "Calibration":
        """Build an *uncalibrated* but usable model from Phil's native grid.

        Uses the ``PLATE_READER`` convention from the existing Squid code
        (A1 at robot (20, 20) mm, 9 mm spacing, columns -> +X, rows A->H
        -> +Y).  The labware's plate-local A1 and spacing are mapped onto
        that convention so ``goto_well`` works before you measure any real
        reference points -- but you should still calibrate for accuracy.
        """
        cal = cls(labware=os.path.basename(getattr(plate, "load_name", "")) or cls.labware)
        a1x, a1y = plate.local_xy("A1")
        col_sp = plate.column_spacing_mm or C.PLATE_READER_SPACING_MM
        row_sp = plate.row_spacing_mm or C.PLATE_READER_SPACING_MM

        # plate-local px increases by col_sp per column; robot X increases by
        # PLATE_READER spacing per column. Assume same magnitude (9 mm).
        sx = C.PLATE_READER_SPACING_MM / col_sp
        # plate-local py DECREASES as the row letter advances (A is at high y),
        # while robot Y INCREASES with the row -> negative slope.
        sy = -C.PLATE_READER_SPACING_MM / row_sp

        cx = C.PLATE_READER_OFFSET_COLUMN_1_MM - sx * a1x
        cy = C.PLATE_READER_OFFSET_ROW_A_MM - sy * a1y
        cal._model_x = (sx, 0.0, cx)
        cal._model_y = (0.0, sy, cy)
        cal._model_z = (0.0, 0.0, C.DEFAULT_Z_WORKING_MM)
        cal.z_safe_mm = C.DEFAULT_Z_SAFE_MM
        cal._rms_error_mm = 0.0
        return cal

    # ------------------------------------------------------------------ fit
    def fit(self, plate: WellPlate) -> None:
        """(Re)fit the transform from the current reference points."""
        pts = self.reference_points
        if not pts:
            self._model_x = self._model_y = self._model_z = None
            return

        # plate-local coordinates of each reference well
        local = np.array([plate.local_xy(p.well) for p in pts], dtype=float)
        robot = np.array([p.robot for p in pts], dtype=float)

        if len(pts) == 1:
            self._fit_translation(local[0], robot[0])
        elif len(pts) == 2:
            self._fit_similarity(local, robot)
        else:
            self._fit_affine(local, robot)

        self._rms_error_mm = self._compute_rms(local, robot)

    def _fit_translation(self, p_local, p_robot) -> None:
        px, py = p_local
        X, Y, Z = p_robot
        self._model_x = (self.axis_sign_x, 0.0, X - self.axis_sign_x * px)
        self._model_y = (0.0, self.axis_sign_y, Y - self.axis_sign_y * py)
        self._model_z = (0.0, 0.0, Z)

    def _fit_similarity(self, local, robot) -> None:
        # Solve robot_xy = s*R*local_xy + t  (rotation + uniform scale + translation)
        (p0x, p0y), (p1x, p1y) = local
        (r0x, r0y, r0z), (r1x, r1y, r1z) = robot
        dp = np.array([p1x - p0x, p1y - p0y])
        dr = np.array([r1x - r0x, r1y - r0y])
        dp_norm2 = float(dp @ dp)
        if dp_norm2 < 1e-9:
            raise ValueError("the two reference wells are too close together to calibrate")
        # complex-number trick: dr/dp gives scale*exp(i*theta)
        a = (dp[0] * dr[0] + dp[1] * dr[1]) / dp_norm2   # = s*cos(theta)
        b = (dp[0] * dr[1] - dp[1] * dr[0]) / dp_norm2   # = s*sin(theta)
        # robot_x = a*px - b*py + cx ; robot_y = b*px + a*py + cy
        cx = r0x - (a * p0x - b * p0y)
        cy = r0y - (b * p0x + a * p0y)
        self._model_x = (a, -b, cx)
        self._model_y = (b, a, cy)
        self._model_z = (0.0, 0.0, 0.5 * (r0z + r1z))

    def _fit_affine(self, local, robot) -> None:
        # Least-squares solve [px py 1] @ coeffs = value, independently for X, Y, Z.
        A = np.column_stack([local[:, 0], local[:, 1], np.ones(len(local))])
        self._model_x = tuple(np.linalg.lstsq(A, robot[:, 0], rcond=None)[0])
        self._model_y = tuple(np.linalg.lstsq(A, robot[:, 1], rcond=None)[0])
        self._model_z = tuple(np.linalg.lstsq(A, robot[:, 2], rcond=None)[0])

    def _compute_rms(self, local, robot) -> float:
        errs = []
        for (px, py), (X, Y, Z) in zip(local, robot):
            mx, my, mz = self._apply(px, py)
            errs.append((mx - X) ** 2 + (my - Y) ** 2 + (mz - Z) ** 2)
        return math.sqrt(sum(errs) / len(errs)) if errs else 0.0

    # -------------------------------------------------------------- transform
    @property
    def is_fitted(self) -> bool:
        return self._model_x is not None

    def _apply(self, px: float, py: float) -> tuple[float, float, float]:
        if not self.is_fitted:
            raise RuntimeError(
                "calibration has no reference points; jog to a well and add one "
                "(see phil.calibrate) before requesting robot coordinates"
            )
        ax, bx, cx = self._model_x
        ay, by, cy = self._model_y
        az, bz, cz = self._model_z
        return (ax * px + bx * py + cx,
                ay * px + by * py + cy,
                az * px + bz * py + cz)

    def well_to_robot(self, plate: WellPlate, well_id: str) -> tuple[float, float, float]:
        """Robot (X, Y, Z) in mm for the center of ``well_id``."""
        px, py = plate.local_xy(well_id)
        X, Y, Z = self._apply(px, py)
        ov = self.well_overrides.get(well_id.strip().upper())
        if ov:
            X += float(ov.get("dx", 0.0))
            Y += float(ov.get("dy", 0.0))
            Z += float(ov.get("dz", 0.0))
        return X, Y, Z

    def set_well_override(self, well_id: str, dx=0.0, dy=0.0, dz=0.0) -> None:
        self.well_overrides[well_id.strip().upper()] = {
            "dx": float(dx), "dy": float(dy), "dz": float(dz)
        }

    def clear_well_override(self, well_id: str) -> None:
        self.well_overrides.pop(well_id.strip().upper(), None)

    # ----------------------------------------------------------- references
    def add_reference(self, well_id: str, robot_xyz, plate: WellPlate | None = None) -> None:
        well_id = well_id.strip().upper()
        self.reference_points = [p for p in self.reference_points if p.well != well_id]
        self.reference_points.append(
            ReferencePoint(well_id, tuple(float(v) for v in robot_xyz))
        )
        if plate is not None:
            self.fit(plate)

    # ---------------------------------------------------------------- persist
    def to_dict(self) -> dict:
        return {
            "labware": self.labware,
            "reference_points": [
                {"well": p.well, "robot": list(p.robot)} for p in self.reference_points
            ],
            "well_overrides": self.well_overrides,
            "z_safe_mm": self.z_safe_mm,
            "z_clearance_mm": self.z_clearance_mm,
            "axis_sign_x": self.axis_sign_x,
            "axis_sign_y": self.axis_sign_y,
            "fit": {
                "model_x": list(self._model_x) if self._model_x else None,
                "model_y": list(self._model_y) if self._model_y else None,
                "model_z": list(self._model_z) if self._model_z else None,
                "rms_error_mm": self._rms_error_mm,
            },
        }

    def save(self, path: str | None = None) -> str:
        path = path or DEFAULT_CALIBRATION
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2)
        return path

    @classmethod
    def from_dict(cls, d: dict, plate: WellPlate | None = None) -> "Calibration":
        cal = cls(
            labware=d.get("labware", "corning_96_wellplate_360ul_flat.json"),
            reference_points=[
                ReferencePoint(p["well"], tuple(p["robot"])) for p in d.get("reference_points", [])
            ],
            well_overrides=d.get("well_overrides", {}),
            z_safe_mm=d.get("z_safe_mm", 0.0),
            z_clearance_mm=d.get("z_clearance_mm", 5.0),
            axis_sign_x=d.get("axis_sign_x", 1.0),
            axis_sign_y=d.get("axis_sign_y", 1.0),
        )
        if plate is not None:
            cal.fit(plate)
        return cal

    @classmethod
    def load(cls, path: str | None = None, plate: WellPlate | None = None) -> "Calibration":
        path = path or DEFAULT_CALIBRATION
        if not os.path.isfile(path):
            return cls()
        with open(path, "r") as f:
            return cls.from_dict(json.load(f), plate=plate)

    def summary(self) -> str:
        n = len(self.reference_points)
        if not self.is_fitted:
            return "calibration: NONE (no reference points set)"
        if n == 0:
            return ("calibration: NOMINAL (uncalibrated PLATE_READER defaults; "
                    "measure reference wells for accuracy)")
        kind = {1: "translation-only", 2: "rotation+scale"}.get(n, "full affine + tilt")
        rms = f"{self._rms_error_mm:.3f} mm" if self._rms_error_mm is not None else "n/a"
        wells = ", ".join(p.well for p in self.reference_points)
        return (f"calibration: {kind} from {n} point(s) [{wells}]; "
                f"fit RMS error {rms}; {len(self.well_overrides)} per-well override(s)")
