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
    # PHIL paper (Open-source personal pipetting robots, Nat. Commun. 13, 2999, 2022):
    # 65 mm proximal arms + 145 mm distal links. The real arm is mildly ASYMMETRIC; the
    # well-constrained 72-well legacy fit measured it as [l1,d1,l2,d2] below. We use that
    # as the prior target + warm-start: it's physical, frame-stable (bases/links don't
    # move; only the ustep<->angle scale rescales with firmware and the offset shifts
    # when home is re-set), and it makes a sparse cross teach recover the true geometry
    # (interior leave-one-out ~0.4 mm) instead of overfitting noise into the under-
    # constrained column-1 link.
    PROXIMAL_MM = 65.0
    DISTAL_MM = 145.0
    NOMINAL_LINKS = (65.44, 140.65, 63.19, 149.02)   # [l1, d1, l2, d2] mm (measured)
    NOMINAL_BASES = (-77.54, 61.76, -80.25, 21.17)   # [b1x, b1y, b2x, b2y] plate-mm
    LEGACY_SCALE = 0.003895                          # |rad/ustep| at legacy full-step scale
    # Soft-prior weight: a residual w*(length - NOMINAL) per link. Pulls the under-
    # constrained links to their measured values without hard bounds (can't diverge to a
    # giant-link solution), while data still shapes everything else.
    LINK_PRIOR_W = 1.0

    def __init__(self):
        self.params = None                 # 12-vector
        self.fk_branch = 1                  # E-from-elbows branch
        self.elbow_br = (1, -1)             # per-arm elbow branch for inverse
        self.zplane = (0.0, 0.0, 0.0)       # az, bz, cz
        self.rms_mm = None
        self.j_ref = None                   # (j1, j2) center of taught joints, for unwrap
        self.ref_jac_sign = 0               # sign(det J) of the chosen assembly mode
        self.ustep_scale = None             # firmware scale the fit was made at (256=v2, 8=legacy)
        self._sign_flips = 0                # taught wells disagreeing with ref_jac_sign
        self._warned_flip = False           # predict() warns once on a mode flip

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
        r1, r2 = (self.j_ref if self.j_ref is not None else (0.0, 0.0))
        return np.array([self._unwrap(j1, s1, r1), self._unwrap(j2, s2, r2)])

    @staticmethod
    def _unwrap(j, s, ref=0.0):
        """Resolve the 2*pi angle-wrap ambiguity by bringing j to within half a
        revolution of ``ref`` (the center of the taught joints). Scale-independent:
        works whether joints are legacy full-steps (+/-hundreds) or v2 microsteps
        (+/-thousands) -- the old hardcoded [-300, 1200] window broke on v2."""
        step = abs(_TWO_PI / s)
        if step <= 0 or not np.isfinite(step):
            return j
        return j - step * round((j - ref) / step)

    # --------------------------------------------------------- assembly mode
    def _jacobian(self, j1, j2, h=5.0):
        """2x2 d(E)/d(joints) by finite difference; None if out of reach."""
        e0 = self.forward(j1, j2)
        ex = self.forward(j1 + h, j2)
        ey = self.forward(j1, j2 + h)
        if e0 is None or ex is None or ey is None:
            return None
        return np.array([[(ex[0] - e0[0]) / h, (ey[0] - e0[0]) / h],
                         [(ex[1] - e0[1]) / h, (ey[1] - e0[1]) / h]])

    def jacobian_sign(self, j1, j2):
        """sign(det J) at a pose. It flips across a Type-2 (parallel) singularity --
        the boundary between the two 5-bar assembly modes -- so a CONSTANT sign over
        the workspace means we never crossed into the mirror solution. 0 = unreachable."""
        Jm = self._jacobian(j1, j2)
        if Jm is None:
            return 0
        det = Jm[0, 0] * Jm[1, 1] - Jm[0, 1] * Jm[1, 0]
        return int(np.sign(det))

    # ------------------------------------------------------------------ fit
    def fit(self, plate, teach_table, n_starts=500, seed=0, refine=True,
            extra_points=None):
        """Fit the 5-bar from taught wells and/or arbitrary (mm <-> joints) points.

        ``extra_points`` is a list of ``(xy_mm, (jX, jY), z_or_None)`` correspondences
        -- e.g. clicked "the outlet is really HERE at these joints" samples from the
        live tracker. Wells and clicks are pooled into one point cloud, so a few of
        either teach the whole grid.
        """
        wells = sorted(teach_table.taught)
        xy = [plate.local_xy(w) for w in wells]
        jj = [[teach_table.taught[w]["X"], teach_table.taught[w]["Y"]] for w in wells]
        zz = [float(teach_table.taught[w]["Z"]) for w in wells]
        for pt in (extra_points or []):
            pxy, pj, pz = pt
            xy.append([float(pxy[0]), float(pxy[1])])
            jj.append([float(pj[0]), float(pj[1])])
            zz.append(float("nan") if pz is None else float(pz))
        XY = np.array(xy, float).reshape(-1, 2)
        J = np.array(jj, float).reshape(-1, 2)
        Z = np.array(zz, float)
        if len(XY) < 3:
            raise ValueError("need >=3 points (taught wells and/or clicked samples) to fit")

        NL, NB, w = self.NOMINAL_LINKS, self.NOMINAL_BASES, self.LINK_PRIOR_W

        def resid_data(par, br, Jm, XYm):
            self.params, self.fk_branch = par, br
            out = []
            for j, xy in zip(Jm, XYm):
                E = self.forward(j[0], j[1])
                out += [1e3, 1e3] if E is None else list(E - xy)
            return out

        def resid(par, br, Jm, XYm):
            # data residuals (mm) + a soft prior pulling the link lengths to the
            # measured NOMINAL geometry. Keeps the fit smooth/unbounded yet unable to
            # diverge (a giant link incurs a large penalty) and recovers the true arm.
            out = resid_data(par, br, Jm, XYm)
            out += [w * (par[4] - NL[0]), w * (par[5] - NL[1]),
                    w * (par[6] - NL[2]), w * (par[7] - NL[3])]
            return out

        def rms_of(x, br, Jm, XYm):
            return float(np.sqrt(np.mean(np.array(resid_data(x, br, Jm, XYm)) ** 2)))

        def per_well(x, br, Jm, XYm):
            self.params, self.fk_branch = x, br
            return np.array([1e3 if (E := self.forward(j[0], j[1])) is None
                             else float(np.hypot(*(E - xy))) for j, xy in zip(Jm, XYm)])

        rng = np.random.default_rng(seed)
        # Scale magnitude is firmware-dependent (legacy full-step ~3.9e-3, v2 microstep
        # ~32x finer); derive it from the joint span so warm-starts begin near the right
        # magnitude for either: ~1 rev sweeps the plate, so |s| ~ 2pi / jspan.
        jspan = max(1.0, float(np.ptp(J)))
        s_seed = _TWO_PI / jspan

        # Warm-starts from the MEASURED nominal geometry: bases + links fixed (physical,
        # frame-stable), scale at the data-derived magnitude (sign unknown -> both), and
        # the home offset re-scanned on a grid (home was re-set). This collapses the
        # non-convex search to offsets+branch and converges reliably (~one shot).
        og = np.linspace(-np.pi, np.pi, 5, endpoint=False)
        warm = [(np.array([NB[0], NB[1], NB[2], NB[3], NL[0], NL[1], NL[2], NL[3],
                           so * s_seed, o1, so * s_seed, o2]), br)
                for br in (1, -1) for so in (1, -1) for o1 in og for o2 in og]

        def rand_start():
            return np.array([
                rng.uniform(-150, 300), rng.uniform(-150, 300),
                rng.uniform(-150, 300), rng.uniform(-150, 300),
                NL[0] + rng.uniform(-4, 4), NL[1] + rng.uniform(-4, 4),
                NL[2] + rng.uniform(-4, 4), NL[3] + rng.uniform(-4, 4),
                rng.choice((-1, 1)) * s_seed * rng.uniform(0.3, 3.0), rng.uniform(-np.pi, np.pi),
                rng.choice((-1, 1)) * s_seed * rng.uniform(0.3, 3.0), rng.uniform(-np.pi, np.pi)])

        def search(Jm, XYm, x_warm=None):
            best = None
            cands = ([(x_warm, 1), (x_warm, -1)] if x_warm is not None else []) + warm
            cands += [(rand_start(), br) for _ in range(n_starts) for br in (1, -1)]
            for x0, br in cands:
                try:
                    sol = least_squares(resid, x0, args=(br, Jm, XYm),
                                        loss="soft_l1", max_nfev=400)
                except Exception:
                    continue
                r = rms_of(sol.x, br, Jm, XYm)
                if best is None or r < best[0]:
                    best = (r, sol.x, br)
            if refine and best is not None:        # polish without robust loss
                try:
                    sol = least_squares(resid, best[1], args=(best[2], Jm, XYm),
                                        loss="linear", max_nfev=2000)
                    r = rms_of(sol.x, best[2], Jm, XYm)
                    if r < best[0] * 1.5:
                        best = (r, sol.x, best[2])
                except Exception:
                    pass
            return best

        best = search(J, XY)
        # Pass 2: outlier rejection. A few hand-taught points carry backlash error (or a
        # mis-centred home corner); drop the worst-fitting and refit on the rest for a
        # cleaner GEOMETRY. The dropped points are still taught -> goto replays them exactly.
        if best is not None and len(XY) >= 8:
            rr = per_well(best[1], best[2], J, XY)
            keep = rr <= max(2.5, 3.0 * float(np.median(rr)))
            if max(4, int(0.75 * len(XY))) <= int(keep.sum()) < len(XY):
                b2 = search(J[keep], XY[keep], x_warm=best[1])
                if b2 is not None and b2[0] <= best[0]:
                    best = b2

        self.rms_mm, self.params, self.fk_branch = best[0], best[1], best[2]
        self.ustep_scale = teach_table.ustep_scale     # 256=v2 microstep, 8=legacy
        self.j_ref = (float(np.mean(J[:, 0])), float(np.mean(J[:, 1])))
        self._pick_elbow_branches(XY, J)
        self._set_ref_sign(J)
        self._fit_zplane(XY, Z)
        return self.rms_mm

    def _pick_elbow_branches(self, XY, J):
        """Choose the per-arm elbow branches that best reproduce ALL points (not just
        the first) -- a single reference can be ambiguous and pick a mode that's wrong
        elsewhere on the plate."""
        best = None
        for b1 in (1, -1):
            for b2 in (1, -1):
                self.elbow_br = (b1, b2)
                errs, ok = [], True
                for E, jt in zip(XY, J):
                    try:
                        errs.append(float(np.hypot(*(self.inverse(np.asarray(E)) - jt))))
                    except ValueError:
                        ok = False
                        break
                if ok and errs:
                    score = float(np.mean(errs))
                    if best is None or score < best[0]:
                        best = (score, (b1, b2))
        if best is not None:
            self.elbow_br = best[1]

    def _set_ref_sign(self, J):
        """Record the assembly-mode sign(det J) shared by the points. A point whose
        sign disagrees signals the fit straddles a singularity (mirror risk) --
        surfaced in summary() and guarded in predict()."""
        signs = [s for jx, jy in J if (s := self.jacobian_sign(jx, jy)) != 0]
        if not signs:
            self.ref_jac_sign, self._sign_flips = 0, 0
            return
        pos = sum(1 for s in signs if s > 0)
        neg = len(signs) - pos
        self.ref_jac_sign = 1 if pos >= neg else -1
        self._sign_flips = min(pos, neg)

    def _fit_zplane(self, XY, Z):
        """Least-squares Z = az*x + bz*y + cz over points that have a Z (clicked
        samples may have none -> NaN, which are dropped). Needs >=3 to fit a plane."""
        m = np.isfinite(Z)
        if int(m.sum()) < 3:
            self.zplane = (0.0, 0.0, 0.0)
            return
        A = np.column_stack([XY[m, 0], XY[m, 1], np.ones(int(m.sum()))])
        coef, *_ = np.linalg.lstsq(A, Z[m], rcond=None)
        self.zplane = tuple(float(c) for c in coef)

    # ------------------------------------------------------------- predict
    @property
    def is_fitted(self):
        return self.params is not None

    def predict(self, well_id, plate, target_plate=None):
        pl = target_plate or plate
        E = np.array(pl.local_xy(well_id), float)
        j = self.inverse(E)
        if self.ref_jac_sign:
            s = self.jacobian_sign(j[0], j[1])
            if s and s != self.ref_jac_sign and not self._warned_flip:
                self._warned_flip = True
                import sys
                print(f"  [kinematics WARNING: {well_id} solves on the OPPOSITE "
                      f"assembly mode (det J sign flip) -- prediction may be mirrored "
                      f"here; verify by eye and re-teach near it if off]", file=sys.stderr)
        az, bz, cz = self.zplane
        zz = az * E[0] + bz * E[1] + cz
        return {"X": int(round(j[0])), "Y": int(round(j[1])), "Z": int(round(zz))}

    # --------------------------------------------------------------- persist
    def to_dict(self):
        return {"params": list(self.params), "fk_branch": int(self.fk_branch),
                "elbow_br": list(self.elbow_br), "zplane": list(self.zplane),
                "rms_mm": self.rms_mm,
                "j_ref": list(self.j_ref) if self.j_ref is not None else None,
                "ref_jac_sign": int(self.ref_jac_sign),
                "ustep_scale": self.ustep_scale}

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
        jr = d.get("j_ref")
        m.j_ref = tuple(jr) if jr is not None else None
        m.ref_jac_sign = int(d.get("ref_jac_sign", 0))
        m.ustep_scale = d.get("ustep_scale")
        return m

    def looks_v2(self):
        """True if this fit was made at v2 microstep scale (so it's safe to use on the
        v2 backend). Prefer the explicit marker; fall back to the angle-scale magnitude
        (v2 ~1e-4 rad/ustep vs legacy ~4e-3) + j_ref for older unmarked files."""
        if not self.is_fitted:
            return False
        if self.ustep_scale == 256:
            return True
        if self.ustep_scale in (8, 1):
            return False
        return self.j_ref is not None and abs(float(self.params[8])) < 1.5e-3

    def summary(self):
        if not self.is_fitted:
            return "kinematics: not fitted"
        p = self.params
        mode = ("fk%+d elbow(%+d,%+d) detJ%+d" %
                (self.fk_branch, self.elbow_br[0], self.elbow_br[1], self.ref_jac_sign))
        warn = ("" if self._sign_flips == 0
                else f"  [!] {self._sign_flips} taught well(s) on the opposite mode -- mirror risk")
        return (f"kinematics: 5-bar fit, RMS {self.rms_mm:.2f} mm over taught wells; "
                f"arms l~{p[4]:.0f}/{p[6]:.0f}mm dist~{p[5]:.0f}/{p[7]:.0f}mm "
                f"(design 65/145); mode {mode}{warn}")
