"""Teach-and-replay well table for the articulated (5-bar) Phil arm.

Because Phil's X/Y are rotary joints on a parallel linkage (not a Cartesian
stage), the safe, measurement-free way to reach a well is to *teach* it: jog
the outlet over the well once and save the joint positions (X, Y, Z motor
positions, in repo microsteps). ``goto`` then replays them.

Untaught wells are estimated by bilinear interpolation in JOINT space from
the four taught corners (A1, A12, H1, H12). That is approximate for a 5-bar
mid-plate, but gives a usable starting point you can refine by teaching more
wells (a taught well always overrides the interpolation).
"""
from __future__ import annotations

import json
import os

from ..paths import DEFAULT_TEACH_PATH  # re-exported so callers keep importing it from here

CORNERS = ("A1", "A12", "H1", "H12")    # default (96-well) corners, for display/messages


def plate_corners(plate) -> tuple:
    """The four geometric corner wells for THIS plate (A1/A12/H1/H12 for a 96,
    A1/A24/P1/P24 for a 384). Use this instead of the hardcoded 96-well CORNERS so
    interpolation/anchor span the real plate."""
    r0, r1 = plate.rows[0], plate.rows[-1]
    c0, c1 = plate.columns[0], plate.columns[-1]
    return (f"{r0}{c0}", f"{r0}{c1}", f"{r1}{c0}", f"{r1}{c1}")


class TeachTable:
    def __init__(self, labware="eppendorf_twintec_lobind_96_pcr.json",
                 z_travel_usteps=None, ustep_scale=None):
        self.labware = labware
        self.taught: dict[str, dict] = {}        # well -> {'X':int,'Y':int,'Z':int}
        self.named: dict[str, dict] = {}         # off-plate positions (WASTE, PARK, ...)
        self.z_travel_usteps = z_travel_usteps   # absolute travel Z (firmware usteps)
        # Marks the joint-count unit the data was taught in: None/8 = legacy
        # full-step firmware, 256 = v2 microstep firmware. goto refuses to replay
        # data taught at the wrong scale (it would be 32x off).
        self.ustep_scale = ustep_scale

    # -------------------------------------------------------------- teaching
    def teach(self, well_id, x, y, z, finish=None):
        """Record a well's joints. ``finish`` = (sx, sy), the per-axis direction the
        operator's LAST nudge engaged (+1 = Up/Right, -1 = Down/Left). goto replays
        the SAME engagement so the count and the physical spot agree despite backlash
        (see finish_for_well). Omit it (None) to leave a well direction-agnostic; goto
        then uses its canonical +X,+Y close-in."""
        rec = {"X": int(round(x)), "Y": int(round(y)), "Z": int(round(z))}
        if finish is not None:
            sx, sy = finish
            rec["finish"] = [1 if (sx is None or sx >= 0) else -1,
                             1 if (sy is None or sy >= 0) else -1]
        self.taught[well_id.strip().upper()] = rec

    def finish_for_well(self, well_id) -> tuple:
        """The backlash finish direction goto must replay for this well, (sx, sy).
        Defaults to (+1, +1) for wells taught before finish was recorded -- i.e. goto's
        original canonical +X,+Y close-in, so nothing regresses."""
        rec = self.taught.get(well_id.strip().upper())
        if rec and "finish" in rec:
            fx, fy = rec["finish"]
            return (1 if fx >= 0 else -1, 1 if fy >= 0 else -1)
        return (1, 1)

    def forget(self, well_id):
        self.taught.pop(well_id.strip().upper(), None)

    def is_taught(self, well_id) -> bool:
        return well_id.strip().upper() in self.taught

    # --------------------------------------------- named (off-plate) positions
    def teach_named(self, name, x, y, z):
        """Save a NAMED off-plate position (e.g. WASTE, PARK) in joint usteps."""
        self.named[name.strip().upper()] = {
            "X": int(round(x)), "Y": int(round(y)), "Z": int(round(z))}

    def forget_named(self, name):
        self.named.pop(name.strip().upper(), None)

    def is_named(self, name) -> bool:
        return name.strip().upper() in self.named

    def joint_for_named(self, name) -> dict:
        return dict(self.named[name.strip().upper()])

    def corners_present(self, plate=None) -> bool:
        corners = plate_corners(plate) if plate is not None else CORNERS
        return all(c in self.taught for c in corners)

    # ----------------------------------------------------------- resolution
    def joint_for_well(self, well_id, plate) -> dict:
        """Return {'X','Y','Z'} joint usteps for a well (taught or interpolated)."""
        w = well_id.strip().upper()
        if w in self.taught:
            return dict(self.taught[w])
        corners = plate_corners(plate)        # real plate corners (96 or 384, etc.)
        if not self.corners_present(plate):
            missing = [c for c in corners if c not in self.taught]
            raise KeyError(
                f"well {w} is not taught and the corners {missing} are not all "
                f"taught yet (need {corners} to interpolate). Teach it directly.")
        row, col = plate.parse_well_id(w)
        n_rows = len(plate.rows)
        n_cols = len(plate.columns)
        u = col / (n_cols - 1) if n_cols > 1 else 0.0   # 0 at first col .. 1 at last
        v = row / (n_rows - 1) if n_rows > 1 else 0.0   # 0 at first row .. 1 at last
        a1, a12, h1, h12 = (self.taught[c] for c in corners)
        out = {}
        for axis in ("X", "Y", "Z"):
            out[axis] = int(round(
                (1 - u) * (1 - v) * a1[axis] + u * (1 - v) * a12[axis]
                + (1 - u) * v * h1[axis] + u * v * h12[axis]))
        return out

    # ------------------------------------------------ rigid-grid learning
    def predict_grid(self, well_id, plate, max_order=2, local=True) -> dict | None:
        """Learn an UNTAUGHT well's joints from the taught wells + the rigid even JSON
        grid -- NOT the 5-bar model (which overfits and put H12 ~50 mm off). The plate
        is an even lattice, so the (col,row) indices ARE the metric coordinate; fit a
        low-order least-squares polynomial surface q=f(col,row) per axis (X/Y/Z) through
        the taught wells and evaluate at the target.

        ``local`` (default) = LOCALLY-WEIGHTED fit: each taught well is weighted by
        1/(dist^2 + 1) in (col,row) cells from the TARGET, so the prediction leans on the
        NEARBY taught wells (the rigid-grid "derive usteps/mm from neighbours" idea) and
        captures the 5-bar's nonlinearity patch-by-patch instead of forcing one global
        plane over the whole plate. Measured on the 8-well L+corners teach this pulls the
        interior leave-one-out from ~3.0 mm (global) to ~1.9 mm -- at the ~1-2 mm hardware
        floor. The eps=1 cell keeps it well-posed in SPARSE regions and makes it degrade
        smoothly toward the global fit when the nearest anchors are far. Pass local=False
        for an unweighted GLOBAL affine (grid_loo uses this -- a weighted fit makes
        held-out CORNER residuals blow up and flag false mis-taught positives).

        Adaptive by COUNT and SPREAD: include a `c`/`r` term only if that coordinate
        actually varies across the taught wells (the collinear/L-shape guard -- an
        all-row-A teach gets no row slope, so off-line predictions fall back to the
        constant instead of a rank-deficient garbage slope); add the bilinear `c*r`
        cross-term at n>=4 (both vary). CAPPED at bilinear (the natural rigid-grid
        interpolant) -- a biquadratic over a sparse scattered teach mis-extrapolates
        corner wells by ~50k usteps. Each solve is column-scaled + conditioning-checked
        on the WEIGHTED matrix; if ill-conditioned it drops one order and refits, down to
        affine. Returns None if <3 wells or still degenerate -- so a SINGLE taught well is
        never predicted here and instead exact-replays via the taught branch. numpy-only.
        """
        import numpy as np
        wells = list(self.taught)
        if len(wells) < 3:
            return None
        cr = np.array([plate.parse_well_id(w)[::-1] for w in wells], float)  # (col,row)
        c, r = cr[:, 0], cr[:, 1]
        c_var = float(c.max() - c.min()) > 1e-9
        r_var = float(r.max() - r.min()) > 1e-9
        if not (c_var or r_var):
            return None                                      # all taught wells one cell
        n = len(wells)
        # CENTER the grid coords before fitting: the raw (col,row,c*r) basis with col up
        # to 11 is badly conditioned, and lstsq then mis-extrapolates at the edges (it put
        # corner wells ~50k usteps off). Centering fixes the conditioning. (Centering is
        # cosmetic for the prediction -- an affine fit's value at the target is the same
        # however the basis is centered -- it only buys conditioning.)
        c0, r0 = float(c.mean()), float(r.mean())
        cs, rs = c - c0, r - r0
        tc, tr = plate.parse_well_id(well_id.strip().upper())[::-1]
        tcs, trs = float(tc) - c0, float(tr) - r0
        terms = {"1": (np.ones(n), 1.0), "c": (cs, tcs), "r": (rs, trs),
                 "cr": (cs * rs, tcs * trs)}
        # Local distance weights (sqrt, to fold into the least-squares rows): lean on the
        # taught wells nearest the TARGET. local=False -> uniform weights = global affine.
        if local:
            d2 = (c - float(tc)) ** 2 + (r - float(tr)) ** 2
            sw = 1.0 / np.sqrt(d2 + 1.0)
        else:
            sw = np.ones(n)

        def basis(order):
            names = ["1"]
            if c_var:
                names.append("c")
            if r_var:
                names.append("r")
            if order >= 2 and c_var and r_var and n >= 4:
                names.append("cr")            # bilinear -- the natural rigid-grid interpolant
            return names

        target_order = min(max_order, 2 if n >= 4 else 1)
        for order in range(target_order, 0, -1):
            names = basis(order)
            Phi = np.column_stack([terms[k][0] for k in names])
            if Phi.shape[0] < Phi.shape[1]:
                continue
            # Column-scale, then SOLVE on the scaled+weighted matrix (not just condition-
            # check it): solving the unscaled basis lets lstsq's rcond drop a singular value
            # and pick a curved solution that mis-extrapolates corner wells by ~50k usteps.
            # Scaling + checking the conditioning of the WEIGHTED matrix keeps it well-posed;
            # drop to a lower order if even then it's ill-conditioned (collinear taught set).
            norms = np.linalg.norm(Phi, axis=0); norms[norms == 0] = 1.0
            Phin = Phi / norms
            Phiw = Phin * sw[:, None]              # row-weighted (local) design matrix
            sv = np.linalg.svd(Phiw, compute_uv=False)
            if sv[-1] <= 1e-6 * sv[0]:
                continue
            xs = np.array([terms[k][1] for k in names], float) / norms
            out = {}
            for axis in ("X", "Y", "Z"):
                q = np.array([self.taught[w][axis] for w in wells], float)
                beta, *_ = np.linalg.lstsq(Phiw, q * sw, rcond=None)
                out[axis] = int(round(float(xs @ beta)))
            return out
        return None

    def grid_loo(self, plate) -> list:
        """Leave-one-out grid residual (XY usteps) per taught well, worst first. Hold a
        well out, predict it from the OTHERS via predict_grid, compare to its taught
        joints. A large residual = a likely MIS-TAUGHT well (the ~50 mm H12 would scream).
        numpy-only, no kin model. Needs >=4 wells (>=3 must remain after holding one out).
        Uses an AFFINE (max_order=1) fit: a bilinear cr-twist extrapolates held-out CORNER
        wells and would flag them as false positives; affine is exact on the rigid grid at
        corners, so only a genuinely off-grid well stands out.
        """
        import numpy as np
        wells = list(self.taught)
        if len(wells) < 4:
            return []
        full = dict(self.taught)
        out = []
        try:
            for w in wells:
                self.taught = {k: v for k, v in full.items() if k != w}
                pred = self.predict_grid(w, plate, max_order=1, local=False)
                if pred is None:
                    continue
                err = float(np.hypot(pred["X"] - full[w]["X"], pred["Y"] - full[w]["Y"]))
                out.append((w, round(err, 1)))
        finally:
            self.taught = full
        return sorted(out, key=lambda t: -t[1])

    # --------------------------------------------------------------- travel
    def travel_z(self, plate=None) -> int | None:
        """Absolute travel Z (usteps). If unset, use the highest taught Z."""
        if self.z_travel_usteps is not None:
            return self.z_travel_usteps
        zs = [v["Z"] for v in self.taught.values()]
        return max(zs) if zs else None

    # ---------------------------------------------------------------- persist
    def to_dict(self) -> dict:
        return {"labware": self.labware,
                "z_travel_usteps": self.z_travel_usteps,
                "ustep_scale": self.ustep_scale,
                "taught": self.taught,
                "named": self.named}

    def save(self, path=None) -> str:
        path = path or DEFAULT_TEACH_PATH
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2)
        return path

    @classmethod
    def load(cls, path=None):
        path = path or DEFAULT_TEACH_PATH
        if not os.path.isfile(path):
            return cls()
        with open(path) as f:
            d = json.load(f)
        t = cls(labware=d.get("labware", "eppendorf_twintec_lobind_96_pcr.json"),
                z_travel_usteps=d.get("z_travel_usteps"),
                ustep_scale=d.get("ustep_scale"))
        t.taught = {k.upper(): v for k, v in d.get("taught", {}).items()}
        t.named = {k.upper(): v for k, v in d.get("named", {}).items()}
        return t

    def summary(self) -> str:
        n = len(self.taught)
        named = (f"; named: {', '.join(sorted(self.named))}" if self.named else "")
        if n == 0:
            return ("teach table: EMPTY (jog to wells and `teach <well>`)" if not self.named
                    else f"teach table: 0 wells taught{named}")
        corners = "corners OK" if self.corners_present() else "corners INCOMPLETE"
        tz = self.travel_z()
        return (f"teach table: {n} well(s) taught [{', '.join(sorted(self.taught))}]; "
                f"{corners}; travel Z={'auto' if self.z_travel_usteps is None else tz}{named}")
