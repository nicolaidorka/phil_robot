"""Neighbour step-count consistency check for taught wells -- per labware.

Wells sit on a regular lattice, so in JOINT space (motor usteps) each taught well
must differ from its grid-neighbours by a predictable step delta. Because X/Y are
rotary 5-bar joints that delta is not globally constant -- it varies smoothly --
so the fitted model supplies the *expected* local delta, and any taught well whose
actual neighbour delta (or whose own joints vs the model) is off by more than the
tolerance has a corrupt teach (e.g. A1 in the current table, ~12 mm out).

The rule is labware-specific by construction: it reads the grid/pitch/adjacency of
the plate the teach table was taught against. Pure logic lives in
``compute_step_violations``; ``main`` is just CLI + report + plot.

    python3 -m phil.stepcheck                       # default = the teach table's labware
    python3 -m phil.stepcheck --tol-mm 2.0
    python3 -m phil.stepcheck --labware <name>      # must match the teach table's plate
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

from .geometry.kinematics import KinematicModel
from .geometry.teach import TeachTable
from .geometry.well_plate import WellPlate, resolve_labware


# --------------------------------------------------------------------- geometry
def _predict_xy(model, plate, well, _cache):
    """Model joints (X,Y usteps) for a well via inverse kinematics, or None.
    Cached because the same well is queried as both a centre and a neighbour."""
    if well not in _cache:
        try:
            j = model.predict(well, plate)
            _cache[well] = np.array([j["X"], j["Y"]], float)
        except (ValueError, KeyError):
            _cache[well] = None
    return _cache[well]


def _nominal_pitch(plate):
    """Center-to-center spacing of adjacent wells, straight from the labware JSON
    (the smallest distance between two distinct well centres)."""
    pts = np.array([plate.local_xy(w) for w in plate.well_ids()], float)
    best = float("inf")
    for i in range(len(pts)):
        d = np.hypot(pts[i, 0] - pts[:, 0], pts[i, 1] - pts[:, 1])
        d[i] = np.inf
        best = min(best, float(d.min()))
    return best if np.isfinite(best) else 9.0


def _grid_neighbours(plate, well, pitch):
    """4-connected neighbours determined from the labware JSON GEOMETRY: wells about
    one pitch away along +/-x or +/-y (axis-aligned), found from their JSON centres
    -- not from well-id naming. Works for any plate, including rotated/odd layouts."""
    x0, y0 = plate.local_xy(well)
    out = []
    for w in plate.well_ids():
        if w == well:
            continue
        dx, dy = (v - v0 for v, v0 in zip(plate.local_xy(w), (x0, y0)))
        if (np.hypot(dx, dy) <= 1.4 * pitch
                and (abs(dx) < 0.3 * pitch or abs(dy) < 0.3 * pitch)):
            out.append(w)
    return out


def _pair_scale(model, plate, a, b, cache):
    """Local usteps-per-mm between adjacent wells a,b: ||Δjoints|| / pitch_mm.
    None if either prediction fails or the pair is degenerate."""
    pa, pb = _predict_xy(model, plate, a, cache), _predict_xy(model, plate, b, cache)
    if pa is None or pb is None:
        return None
    (xa, ya), (xb, yb) = plate.local_xy(a), plate.local_xy(b)
    pitch = float(np.hypot(xb - xa, yb - ya))
    if pitch < 1e-6:
        return None
    scale = float(np.hypot(*(pb - pa)) / pitch)
    return scale if scale > 1e-6 else None


def _local_scale(model, plate, well, cache, global_scale, pitch):
    """Deterministic local usteps-per-mm at ``well``: mean over its JSON neighbours
    (taught or not). Falls back to the plate-wide median."""
    vals = [s for nb in _grid_neighbours(plate, well, pitch)
            if (s := _pair_scale(model, plate, well, nb, cache)) is not None]
    return float(np.mean(vals)) if vals else global_scale


def _global_scale(model, plate, pitch, cache):
    """Plate-wide median usteps-per-mm over all adjacent (JSON) well pairs."""
    scales = []
    for w in plate.well_ids():
        for nb in _grid_neighbours(plate, w, pitch):
            s = _pair_scale(model, plate, w, nb, cache)
            if s is not None:
                scales.append(s)
    return float(np.median(scales)) if scales else 1.0


def _score_well(plate, teach, model, well, pitch, global_scale, cache):
    """Neighbourhood step-count score for ONE taught well: how far its joints sit
    from what its taught JSON-neighbours imply (mm), with a single-well fallback to
    the model so isolated wells are still scored. Reused live (lock a well) and in
    the batch report."""
    tw = np.array([teach.taught[well]["X"], teach.taught[well]["Y"]], float)
    pw = _predict_xy(model, plate, well, cache)
    nb_mm, nb_resid_us, n_nb = 0.0, 0.0, 0
    for nb in _grid_neighbours(plate, well, pitch):
        if nb not in teach.taught:
            continue
        s = _pair_scale(model, plate, well, nb, cache)
        pnb = _predict_xy(model, plate, nb, cache)
        if s is None or pw is None or pnb is None:
            continue
        n_nb += 1
        tnb = np.array([teach.taught[nb]["X"], teach.taught[nb]["Y"]], float)
        resid_us = float(np.hypot(*((tnb - tw) - (pnb - pw))))
        resid_mm = resid_us / s
        if resid_mm > nb_mm:
            nb_mm, nb_resid_us = resid_mm, resid_us
    scale = _local_scale(model, plate, well, cache, global_scale, pitch)
    single_mm = float(np.hypot(*(tw - pw)) / scale) if pw is not None else float("nan")
    score = max(nb_mm, 0.0 if np.isnan(single_mm) else single_mm)
    return {"well": well, "score_mm": score, "nb_mm": nb_mm, "single_mm": single_mm,
            "nb_resid_us": nb_resid_us, "n_neighbours": n_nb}


def neighbour_score(plate, teach, model, well, tol_mm=2.5):
    """Score a single taught well against its JSON neighbourhood (for live use the
    instant a well is locked). Returns the _score_well dict plus a ``flag``."""
    cache: dict = {}
    pitch = _nominal_pitch(plate)
    gs = _global_scale(model, plate, pitch, cache)
    r = _score_well(plate, teach, model, well, pitch, gs, cache)
    r["flag"] = r["score_mm"] > tol_mm
    return r


# ------------------------------------------------------------------- the rule
def compute_step_violations(plate, teach, model, tol_mm=2.5):
    """Score every taught well by how far its joints break the neighbour
    step-count rule. Returns a dict with the per-well rows + diagnostics."""
    cache: dict[str, object] = {}
    taught = teach.taught
    wells = [w for w in sorted(taught) if w in plate]
    dropped = sorted(w for w in taught if w not in plate)
    pitch = _nominal_pitch(plate)

    # plate-wide scale + per-step ranges, over ALL adjacent (JSON) pairs (each once)
    all_scales, col_steps, row_steps = [], [], []
    for w in plate.well_ids():
        x0, y0 = plate.local_xy(w)
        pw = _predict_xy(model, plate, w, cache)
        if pw is None:
            continue
        for nb in _grid_neighbours(plate, w, pitch):
            pnb = _predict_xy(model, plate, nb, cache)
            if pnb is None:
                continue
            dx, dy = (v - v0 for v, v0 in zip(plate.local_xy(nb), (x0, y0)))
            along_col = abs(dx) >= abs(dy)
            if (along_col and dx <= 0) or (not along_col and dy <= 0):
                continue                       # +x / +y only -> count each pair once
            (col_steps if along_col else row_steps).append(pnb - pw)
            s = _pair_scale(model, plate, w, nb, cache)
            if s is not None:
                all_scales.append(s)
    global_scale = float(np.median(all_scales)) if all_scales else 1.0

    rows = []
    for w in wells:
        r = _score_well(plate, teach, model, w, pitch, global_scale, cache)
        r["flag"] = r["score_mm"] > tol_mm
        rows.append(r)

    rows.sort(key=lambda r: -r["score_mm"])
    col_steps = np.array(col_steps) if col_steps else np.empty((0, 2))
    row_steps = np.array(row_steps) if row_steps else np.empty((0, 2))
    return {"rows": rows, "dropped": dropped, "tol_mm": tol_mm,
            "global_scale": global_scale, "col_steps": col_steps,
            "row_steps": row_steps}


# ---------------------------------------------------------------------- report
def _print_report(res, plate):
    cs, rs = res["col_steps"], res["row_steps"]
    print(f"per-step counts ({plate.load_name}, model usteps per 1-well move):")
    if len(cs):
        print(f"  +1 column: X {cs[:,0].min():+.0f}..{cs[:,0].max():+.0f}  "
              f"Y {cs[:,1].min():+.0f}..{cs[:,1].max():+.0f}")
    if len(rs):
        print(f"  +1 row:    X {rs[:,0].min():+.0f}..{rs[:,0].max():+.0f}  "
              f"Y {rs[:,1].min():+.0f}..{rs[:,1].max():+.0f}")
    print(f"  local scale ~{res['global_scale']:.1f} usteps/mm (plate median)\n")

    print(f"neighbour step-count rule (tol {res['tol_mm']} mm), worst first:")
    print(f"  {'well':>4} {'score':>7} {'nb_mm':>7} {'single':>7} {'nb_resid_us':>12}  flag")
    for r in res["rows"]:
        sgl = "   nan" if np.isnan(r["single_mm"]) else f"{r['single_mm']:7.2f}"
        print(f"  {r['well']:>4} {r['score_mm']:7.2f} {r['nb_mm']:7.2f} {sgl} "
              f"{r['nb_resid_us']:12.1f}  {'YES' if r['flag'] else ''}")
    bad = [r["well"] for r in res["rows"] if r["flag"]]
    print(f"\n{len(bad)} well(s) break the rule: {', '.join(bad) if bad else '(none)'}")
    if res["dropped"]:
        print(f"[skipped {len(res['dropped'])} taught well(s) not on this labware: "
              f"{', '.join(res['dropped'])}]")


def _save_plot(res, plate, out_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from .viz import draw_plate_grid

    tol = res["tol_mm"]
    fig, ax = plt.subplots(figsize=(13, 9))
    draw_plate_grid(ax, plate, edgecolor="#cccccc", lw=0.8, alpha=1.0)

    xs, ys, vals = [], [], []
    for r in res["rows"]:
        x, y = plate.local_xy(r["well"])
        xs.append(x); ys.append(y); vals.append(r["score_mm"])
        if r["flag"]:
            ax.annotate(f"{r['well']}\n{r['score_mm']:.1f}mm", (x, y),
                        textcoords="offset points", xytext=(7, 7),
                        fontsize=8, color="#b30000", weight="bold")
    sc = ax.scatter(xs, ys, c=vals, cmap="RdYlGn_r", vmin=0, vmax=tol * 1.5,
                    s=90, marker="x", linewidths=2.5, zorder=4)
    fig.colorbar(sc, ax=ax, label="step-count rule error (mm)")
    ax.set_title(f"Neighbour step-count rule -- {plate.load_name} (tol {tol} mm)\n"
                 f"red = breaks the rule (joints inconsistent with the lattice)")
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    print(f"\nsaved {out_path}")


def main(argv=None):
    raw = list(argv if argv is not None else sys.argv[1:])
    ap = argparse.ArgumentParser(prog="phil.stepcheck",
                                 description="Per-labware neighbour step-count check of taught wells.")
    ap.add_argument("--labware", default=None,
                    help="labware name/path; defaults to the teach table's own plate")
    ap.add_argument("--tol-mm", type=float, default=2.5, dest="tol_mm")
    ap.add_argument("--out", default=None, help="output PNG path")
    args = ap.parse_args(raw)

    teach = TeachTable.load()
    if not teach.taught:
        print("teach table is empty -- nothing to check.")
        return 1

    labware = args.labware or teach.labware
    # guard: checking joints against a different plate's grid is meaningless
    if args.labware is not None:
        try:
            if os.path.realpath(resolve_labware(args.labware)) != \
               os.path.realpath(resolve_labware(teach.labware)):
                print(f"--labware {args.labware!r} differs from the teach table's plate "
                      f"({teach.labware!r}); the joints were taught against that plate. "
                      f"Re-run without --labware, or pass the matching plate.")
                return 1
        except FileNotFoundError as e:
            print(e); return 1

    plate = WellPlate.load(labware)
    model = KinematicModel.load()
    if model is None or not model.is_fitted:
        print("no fitted kinematics model (phil_kinematics.json) -- run fitkin first.")
        return 1

    res = compute_step_violations(plate, teach, model, tol_mm=args.tol_mm)
    _print_report(res, plate)
    out = args.out or f"stepcheck_{plate.load_name}.png"
    _save_plot(res, plate, out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
