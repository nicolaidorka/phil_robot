"""5-bar parallel-arm kinematic model for the Phil robot.

Phil's two motors each drive a proximal link; the two distal links meet at the
outlet. So the outlet position E is a closed-form function of the two motor
angles (forward kinematics), and the motor angles are a closed-form function of
E (inverse kinematics) -- standard 5-bar geometry.

We fit the geometry (pivot positions, link lengths, and the ustep<->angle
scale/offset of each motor) from the taught wells: plate-local mm (E, known
from the labware JSON) <-> measured joint usteps. Once fitted, ANY well on ANY
plate is computed from its JSON mm via inverse kinematics -- accurately and
uniformly across the whole plate, with no interpolation sag.

Model (all lengths in plate-local mm, angles in radians):
    elbow_i = base_i + l_i * (cos th_i, sin th_i),   th_i = s_i * joint_i + o_i
    E       = intersection of circle(elbow_1, dist_1) and circle(elbow_2, dist_2)
Parameters (12): base1(2), base2(2), l1, dist1, l2, dist2, s1, o1, s2, o2.
Z is handled separately by a small tilt plane (Z = az*x + bz*y + cz).
"""
from __future__ import annotations

import json
import os

import numpy as np
from scipy.optimize import least_squares

from ..paths import DEFAULT_KIN_PATH
_TWO_PI = 2.0 * np.pi


def _circ(P1, r1, P2, r2, br):
    """One intersection of two circles, branch br in {+1,-1}; None if disjoint."""
    d = P2 - P1
    D = float(np.hypot(d[0], d[1]))
    if D < 1e-9 or D > r1 + r2 or D < abs(r1 - r2):
        return None
    a = (r1 * r1 - r2 * r2 + D * D) / (2 * D)
    h2 = r1 * r1 - a * a
    if h2 < 0:
        return None
    h = np.sqrt(h2)
    M = P1 + a * d / D
    perp = np.array([-d[1], d[0]]) / D
    return M + br * h * perp


class KinematicModel:
    def __init__(self):
        self.params = None                 # 12-vector
        self.fk_branch = 1                  # E-from-elbows branch
        self.elbow_br = (1, -1)             # per-arm elbow branch for inverse
        self.zplane = (0.0, 0.0, 0.0)       # az, bz, cz
        self.rms_mm = None

    # ------------------------------------------------------------- geometry
    def _bases_links(self):
        p = self.params
        return (np.array([p[0], p[1]]), np.array([p[2], p[3]]),
                p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11])

    def forward(self, j1, j2):
        B1, B2, l1, d1, l2, d2, s1, o1, s2, o2 = self._bases_links()
        th1, th2 = s1 * j1 + o1, s2 * j2 + o2
        e1 = B1 + l1 * np.array([np.cos(th1), np.sin(th1)])
        e2 = B2 + l2 * np.array([np.cos(th2), np.sin(th2)])
        return _circ(e1, d1, e2, d2, self.fk_branch)

    def inverse(self, E):
        """Plate-local (x, y) mm -> (joint_X, joint_Y) usteps."""
        B1, B2, l1, d1, l2, d2, s1, o1, s2, o2 = self._bases_links()
        e1 = _circ(B1, l1, E, d1, self.elbow_br[0])
        e2 = _circ(B2, l2, E, d2, self.elbow_br[1])
        if e1 is None or e2 is None:
            raise ValueError(f"position {E} is outside the arm's reach")
        th1 = np.arctan2(*(e1 - B1)[::-1])
        th2 = np.arctan2(*(e2 - B2)[::-1])
        j1 = (th1 - o1) / s1
        j2 = (th2 - o2) / s2
        return np.array([self._unwrap(j1, s1), self._unwrap(j2, s2)])

    @staticmethod
    def _unwrap(j, s):
        """Bring the angle-wrap ambiguity into the plausible joint range."""
        step = abs(_TWO_PI / s)
        while j < -300:
            j += step
        while j > 1200:
            j -= step
        return j

    # ------------------------------------------------------------------ fit
    def fit(self, plate, teach_table, n_starts=500, seed=0, refine=True):
        wells = sorted(teach_table.taught)
        XY = np.array([plate.local_xy(w) for w in wells], float)
        J = np.array([[teach_table.taught[w]["X"], teach_table.taught[w]["Y"]]
                      for w in wells], float)

        def resid(par, br):
            self.params, self.fk_branch = par, br
            out = []
            for j, xy in zip(J, XY):
                E = self.forward(j[0], j[1])
                out += [1e3, 1e3] if E is None else list(E - xy)
            return out

        rng = np.random.default_rng(seed)
        best = None
        for br in (1, -1):
            for _ in range(n_starts):
                x0 = np.array([
                    rng.uniform(-150, 300), rng.uniform(-150, 300),
                    rng.uniform(-150, 300), rng.uniform(-150, 300),
                    rng.uniform(40, 250), rng.uniform(40, 250),
                    rng.uniform(40, 250), rng.uniform(40, 250),
                    rng.uniform(-0.002, 0.002), rng.uniform(-np.pi, np.pi),
                    rng.uniform(-0.002, 0.002), rng.uniform(-np.pi, np.pi)])
                try:
                    sol = least_squares(resid, x0, args=(br,), loss="soft_l1", max_nfev=400)
                except Exception:
                    continue
                rms = float(np.sqrt(np.mean(np.array(resid(sol.x, br)) ** 2)))
                if best is None or rms < best[0]:
                    best = (rms, sol.x, br)
        if refine and best is not None:           # polish without robust loss
            try:
                sol = least_squares(resid, best[1], args=(best[2],), loss="linear", max_nfev=2000)
                rms = float(np.sqrt(np.mean(np.array(resid(sol.x, best[2])) ** 2)))
                if rms < best[0] * 1.5:
                    best = (rms, sol.x, best[2])
            except Exception:
                pass

        self.rms_mm, self.params, self.fk_branch = best[0], best[1], best[2]
        self._pick_elbow_branches(plate, teach_table)
        self._fit_zplane(plate, teach_table)
        return self.rms_mm

    def _pick_elbow_branches(self, plate, teach_table):
        ref = sorted(teach_table.taught)[0]
        E = np.array(plate.local_xy(ref))
        jt = np.array([teach_table.taught[ref]["X"], teach_table.taught[ref]["Y"]], float)
        best = None
        for b1 in (1, -1):
            for b2 in (1, -1):
                self.elbow_br = (b1, b2)
                try:
                    jj = self.inverse(E)
                except ValueError:
                    continue
                err = float(np.hypot(*(jj - jt)))
                if best is None or err < best[0]:
                    best = (err, (b1, b2))
        self.elbow_br = best[1]

    def _fit_zplane(self, plate, teach_table):
        wells = sorted(teach_table.taught)
        A = np.array([[*plate.local_xy(w), 1.0] for w in wells])
        z = np.array([teach_table.taught[w]["Z"] for w in wells], float)
        coef, *_ = np.linalg.lstsq(A, z, rcond=None)
        self.zplane = tuple(float(c) for c in coef)

    # ------------------------------------------------------------- predict
    @property
    def is_fitted(self):
        return self.params is not None

    def predict(self, well_id, plate, target_plate=None):
        pl = target_plate or plate
        E = np.array(pl.local_xy(well_id), float)
        j = self.inverse(E)
        az, bz, cz = self.zplane
        zz = az * E[0] + bz * E[1] + cz
        return {"X": int(round(j[0])), "Y": int(round(j[1])), "Z": int(round(zz))}

    # --------------------------------------------------------------- persist
    def to_dict(self):
        return {"params": list(self.params), "fk_branch": int(self.fk_branch),
                "elbow_br": list(self.elbow_br), "zplane": list(self.zplane),
                "rms_mm": self.rms_mm}

    def save(self, path=None):
        path = path or DEFAULT_KIN_PATH
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2)
        return path

    @classmethod
    def load(cls, path=None):
        path = path or DEFAULT_KIN_PATH
        if not os.path.isfile(path):
            return None
        with open(path) as f:
            d = json.load(f)
        m = cls()
        m.params = np.array(d["params"])
        m.fk_branch = d.get("fk_branch", 1)
        m.elbow_br = tuple(d.get("elbow_br", (1, -1)))
        m.zplane = tuple(d.get("zplane", (0, 0, 0)))
        m.rms_mm = d.get("rms_mm")
        return m

    def summary(self):
        if not self.is_fitted:
            return "kinematics: not fitted"
        return f"kinematics: 5-bar fit, RMS {self.rms_mm:.2f} mm over taught wells"
