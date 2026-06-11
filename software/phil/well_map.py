"""Nonlinear well->joint map for the articulated Phil arm.

The 5-bar arm is too curved for a single affine (straight-line) map, so this
fits a smooth radial-basis-function (RBF) surface through the taught wells:
plate-local (x, y) mm  ->  joint (X, Y, Z) usteps.

Because the arm is smooth, RBF interpolation passes through the taught points
and curves sensibly between them. It also generalizes to other labware: a new
plate's well mm coordinates (from its JSON) go through the same surface.

A leave-one-out (LOO) check flags any taught well whose joints disagree with
what the other wells predict -- i.e. a likely mis-taught point.
"""
from __future__ import annotations

import numpy as np
from scipy.interpolate import RBFInterpolator


class WellMap:
    def __init__(self, plate, teach_table, smoothing: float = 0.0):
        self.plate = plate
        self.teach = teach_table
        self.smoothing = smoothing
        self._rbf = None
        self._wells: list[str] = []

    @property
    def n(self) -> int:
        return len(self._wells)

    def _data(self, plate=None):
        pl = plate or self.plate
        wells = sorted(self.teach.taught.keys())
        pts = np.array([pl.local_xy(w) for w in wells], dtype=float)
        vals = np.array([[self.teach.taught[w]["X"],
                          self.teach.taught[w]["Y"],
                          self.teach.taught[w]["Z"]] for w in wells], dtype=float)
        return wells, pts, vals

    def fit(self) -> bool:
        wells, pts, vals = self._data()
        if len(wells) < 4:
            self._rbf = None
            self._wells = wells
            return False
        kernel = "thin_plate_spline" if len(wells) >= 5 else "linear"
        self._rbf = RBFInterpolator(pts, vals, kernel=kernel, smoothing=self.smoothing)
        self._wells = wells
        return True

    @property
    def is_fitted(self) -> bool:
        return self._rbf is not None

    def predict(self, well_id: str, plate=None) -> dict:
        pl = plate or self.plate
        xy = np.array([pl.local_xy(well_id)], dtype=float)
        v = self._rbf(xy)[0]
        return {"X": int(round(v[0])), "Y": int(round(v[1])), "Z": int(round(v[2]))}

    # ---------------------------------------------------------- diagnostics
    def loo_errors(self) -> list[tuple[str, float]]:
        """Leave-one-out joint error (usteps) per taught well, worst first."""
        wells, pts, vals = self._data()
        if len(wells) < 5:
            return []
        out = []
        kernel = "thin_plate_spline"
        for i, w in enumerate(wells):
            mask = np.arange(len(wells)) != i
            rbf = RBFInterpolator(pts[mask], vals[mask], kernel=kernel,
                                  smoothing=self.smoothing)
            pred = rbf(pts[i:i + 1])[0]
            err = float(np.linalg.norm(pred[:2] - vals[i][:2]))  # XY only
            out.append((w, err))
        return sorted(out, key=lambda t: -t[1])

    def summary(self) -> str:
        if not self.is_fitted:
            return f"well map: NOT fitted ({self.n} wells; need >=4)"
        loo = self.loo_errors()
        if loo:
            worst = ", ".join(f"{w}:{e:.0f}" for w, e in loo[:3])
            return (f"well map: RBF over {self.n} wells; "
                    f"leave-one-out XY error (usteps) worst: {worst}")
        return f"well map: RBF over {self.n} wells (need >=5 for LOO check)"
