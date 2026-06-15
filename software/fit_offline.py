"""Offline robust fitter for Phil's 5-bar from the REAL taught wells (no hardware).
Warm-starts from the legacy physical geometry (bases+links unchanged; scale /32 for
v2; offsets re-searched because home was re-set), so the non-convex fit converges
reliably. Saves the best model immediately, then reports per-well residuals (to spot
mis-taught wells) and a fast leave-one-out generalization estimate.
"""
import json
import numpy as np
from scipy.optimize import least_squares

from phil.geometry.kinematics import KinematicModel, _TWO_PI
from phil.geometry.well_plate import WellPlate
from phil.geometry.teach import TeachTable

plate = WellPlate.load()
tt = TeachTable.load()
wells = sorted(tt.taught)
XY = np.array([plate.local_xy(w) for w in wells], float)
J = np.array([[tt.taught[w]["X"], tt.taught[w]["Y"]] for w in wells], float)
print(f"loaded {len(wells)} taught wells, ustep_scale={tt.ustep_scale}", flush=True)

LEG_BASES = [-77.54, 61.76, -80.25, 21.17]
LEG_LINKS = [65.44, 140.65, 63.19, 149.02]
S_V2 = 0.003895 / 32.0
# prior target = legacy per-link geometry (from the well-constrained 72-well fit),
# more physical than idealized 65/145. [l1, d1, l2, d2]
LINK_TGT = [65.44, 140.65, 63.19, 149.02]
PRIOR_W = 1.0
L, D = 65.0, 145.0  # (kept for any leftover refs)


def make_resid(Jm, XYm, br_holder):
    def resid(par):
        m = br_holder["m"]; m.params, m.fk_branch = par, br_holder["br"]
        out = []
        for j, xy in zip(Jm, XYm):
            E = m.forward(j[0], j[1])
            out += [1e3, 1e3] if E is None else list(E - xy)
        out += [PRIOR_W * (par[4] - LINK_TGT[0]), PRIOR_W * (par[5] - LINK_TGT[1]),
                PRIOR_W * (par[6] - LINK_TGT[2]), PRIOR_W * (par[7] - LINK_TGT[3])]
        return out
    return resid


def fit_from(x0, br, Jm, XYm):
    m = KinematicModel(); holder = {"m": m, "br": br}
    resid = make_resid(Jm, XYm, holder)
    try:
        sol = least_squares(resid, x0, loss="soft_l1", max_nfev=600)
        sol = least_squares(resid, sol.x, loss="linear", max_nfev=2000)
    except Exception:
        return None, 1e9
    m.params, m.fk_branch = sol.x, br
    r = []
    for j, xy in zip(Jm, XYm):
        E = m.forward(j[0], j[1]); r += [1e3, 1e3] if E is None else list(E - xy)
    return sol.x, float(np.sqrt(np.mean(np.array(r) ** 2)))


def warmstarts(Jm, rng, n_rand=24, ogrid=5):
    cands = []
    for br in (1, -1):
        for so in (1, -1):
            for o1 in np.linspace(-np.pi, np.pi, ogrid, endpoint=False):
                for o2 in np.linspace(-np.pi, np.pi, ogrid, endpoint=False):
                    cands.append((np.array(LEG_BASES + LEG_LINKS +
                                  [so * S_V2, o1, so * S_V2, o2]), br))
    jspan = max(1.0, float(np.ptp(Jm))); s = _TWO_PI / jspan
    for _ in range(n_rand):
        for br in (1, -1):
            cands.append((np.array([rng.uniform(-150, 300), rng.uniform(-150, 300),
                rng.uniform(-150, 300), rng.uniform(-150, 300),
                L + rng.uniform(-4, 4), D + rng.uniform(-4, 4),
                L + rng.uniform(-4, 4), D + rng.uniform(-4, 4),
                rng.choice((-1, 1)) * s * rng.uniform(.3, 3), rng.uniform(-np.pi, np.pi),
                rng.choice((-1, 1)) * s * rng.uniform(.3, 3), rng.uniform(-np.pi, np.pi)]), br))
    return cands


def best_fit(Jm, XYm, rng, n_rand=24, ogrid=5):
    best = None
    for x0, br in warmstarts(Jm, rng, n_rand, ogrid):
        x, rms = fit_from(x0, br, Jm, XYm)
        if x is not None and (best is None or rms < best[1]):
            best = (x, rms, br)
    return best


rng = np.random.default_rng(12345)
x, rms, br = best_fit(J, XY, rng)
print(f"\nBEST taught RMS: {rms:.3f} mm  fk_branch={br}")
print(f"  arms l~{x[4]:.1f}/{x[6]:.1f}  dist~{x[5]:.1f}/{x[7]:.1f}  s1={x[8]:.3g} s2={x[10]:.3g}", flush=True)

# save immediately so goto/sweep use it even if the rest is slow
m = KinematicModel(); m.params = x; m.fk_branch = br; m.rms_mm = rms
m.j_ref = (float(np.mean(J[:, 0])), float(np.mean(J[:, 1])))
m._pick_elbow_branches(plate, tt); m._set_ref_sign(plate, tt); m._fit_zplane(plate, tt)
print("saved ->", m.save())
print(m.summary(), flush=True)

# per-well residuals (tip mm) -> spot mis-taught wells
print("\nper-well fit residual (tip mm), worst first:")
res = []
for w, xy, j in zip(wells, XY, J):
    E = m.forward(j[0], j[1]); res.append((w, float(np.hypot(*(E - xy))) if E is not None else 99))
for w, e in sorted(res, key=lambda t: -t[1]):
    flag = "  <-- likely mis-taught" if e > 2.0 else ""
    print(f"  {w:4s} {e:5.2f} mm{flag}")

def loo_eval(exclude=()):
    """Honest LOO: drop each well, predict ITS joints, compare to the well's TAUGHT
    joints (hardware ground truth). Convert joint error -> mm via the model Jacobian
    (mm per ustep) at that pose. `exclude` wells are dropped from EVERY fit (treated
    as mis-taught) and not scored."""
    keep = [k for k in range(len(wells)) if wells[k] not in exclude]
    out = []
    for i in keep:
        mask = [k for k in keep if k != i]
        bx, _ = fit_from(x, br, J[mask], XY[mask])      # warm-start from global best
        if bx is None:
            continue
        mm = KinematicModel(); mm.params = bx; mm.fk_branch = br
        mm.j_ref = (float(np.mean(J[mask, 0])), float(np.mean(J[mask, 1])))
        ttm = TeachTable(); ttm.ustep_scale = 256
        for k in mask:
            ttm.teach(wells[k], J[k, 0], J[k, 1], 0)
        mm._pick_elbow_branches(plate, ttm)
        try:
            pj = mm.predict(wells[i], plate)
            dj = np.array([pj["X"] - J[i, 0], pj["Y"] - J[i, 1]], float)
            Jac = mm._jacobian(pj["X"], pj["Y"])         # d(mm)/d(joint), 2x2
            mm_err = float(np.hypot(*(Jac @ dj))) if Jac is not None else np.nan
            out.append((wells[i], mm_err))
        except Exception:
            out.append((wells[i], np.nan))
    e = np.array([v for _, v in out if v == v])
    wst = max((t for t in out if t[1] == t[1]), key=lambda t: t[1])
    return e, wst, out


CORNERS = {"A1", "A12", "H1", "H12"}


def score(out):
    """LOO over NON-corner wells only -- corners are always taught/replayed, so their
    held-out extrapolation error is irrelevant to real use; interior is what counts."""
    v = np.array([e for w, e in out if w not in CORNERS and e == e])
    return float(np.median(v)), float(np.mean(v)), float(np.max(v))


print("\nleave-one-out (predict held-out joints vs taught; mm via Jacobian):", flush=True)
_, _, out_all = loo_eval()
med, mean, mx = score(out_all)
print(f"  fit ALL 24:        interior LOO  median {med:.2f}  mean {mean:.2f}  worst {mx:.2f} mm", flush=True)

# outlier set: A1 (home corner, big residual) + any taught well whose fit residual >2.5mm
outliers = tuple(sorted({"A1"} | {w for w, e in res if e > 2.5}))
_, _, out_ex = loo_eval(exclude=outliers)
med2, mean2, mx2 = score(out_ex)
print(f"  fit excl {outliers}:")
print(f"                     interior LOO  median {med2:.2f}  mean {mean2:.2f}  worst {mx2:.2f} mm", flush=True)

# Save whichever geometry generalizes better on the interior. Refit on the kept wells.
if med2 < med:
    keep = [k for k in range(len(wells)) if wells[k] not in outliers]
    bx, brms = fit_from(x, br, J[keep], XY[keep])
    mm2 = KinematicModel(); mm2.params = bx; mm2.fk_branch = br; mm2.rms_mm = brms
    mm2.j_ref = (float(np.mean(J[keep, 0])), float(np.mean(J[keep, 1])))
    tkeep = TeachTable(); tkeep.ustep_scale = 256
    for k in keep:
        tkeep.teach(wells[k], J[k, 0], J[k, 1], 0)
    mm2._pick_elbow_branches(plate, tkeep); mm2._set_ref_sign(plate, tkeep)
    mm2._fit_zplane(plate, tt)
    mm2.save()
    print(f"\nSAVED outlier-excluded geometry (better interior LOO {med2:.2f} vs {med:.2f} mm).")
    print(" ", mm2.summary())
    print(f"  NOTE: excluded {outliers} from the FIT only -- they still replay exactly on goto.")
    print(f"  If you want them sharper as untaught references too, re-teach them (center carefully,")
    print(f"  finish Up/Right). A1 is home -- recenter it before pressing h if you reteach.")
else:
    print(f"\nKEPT all-24 geometry (already best interior LOO {med:.2f} mm).")
