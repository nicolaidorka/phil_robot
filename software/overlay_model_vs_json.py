"""Overlay the current 5-bar kinematics model on the nominal 96-well plate grid.

Blue  = nominal well centres straight from the labware JSON (ground truth).
Red   = where the MODEL reproduces each well:
          - taught well  -> forward-kinematics of its recorded joints
                            (this is the real fit residual vs nominal)
          - untaught well-> model-predicted joints, forward-kinematics
Lines  = per-well error vector (exaggerated x10 so sub-mm error is visible).
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from phil.geometry.well_plate import WellPlate
from phil.geometry.kinematics import KinematicModel
from phil.geometry.teach import TeachTable

ERR_GAIN = 1.0  # 1.0 = true scale; raise to exaggerate sub-mm error for visibility

plate = WellPlate.load()
model = KinematicModel.load()
teach = TeachTable.load()

wells = plate.well_ids()
nom = {w: np.array(plate.local_xy(w), float) for w in wells}

model_xy = {}
status = {}
for w in wells:
    if teach.is_taught(w):
        jt = teach.taught[w]
        E = model.forward(jt["X"], jt["Y"])   # FK of the REAL recorded joints
        status[w] = "taught"
    else:
        j = model.predict(w, plate)           # model inverse -> joints
        E = model.forward(j["X"], j["Y"])     # ... and back to mm
        status[w] = "model"
    if E is not None:
        model_xy[w] = np.array(E, float)

# --- residual stats (taught wells only -- the real measurement) -------------
err = {w: float(np.hypot(*(model_xy[w] - nom[w])))
       for w in wells if w in model_xy}
taught_err = [err[w] for w in wells if status[w] == "taught" and w in err]
rms = float(np.sqrt(np.mean(np.square(taught_err)))) if taught_err else float("nan")
mx = max(taught_err) if taught_err else float("nan")

print("\nper-taught-well error (mm), worst first:")
for w in sorted((w for w in err if status[w] == "taught"),
                key=lambda w: -err[w]):
    print(f"  {w:>4}  {err[w]:6.2f}")

# --- plot -------------------------------------------------------------------
from phil.viz import draw_plate_grid

fig, ax = plt.subplots(figsize=(13, 9))
draw_plate_grid(ax, plate, label_corners=False, set_frame=False)  # shared well circles

# nominal centres
nx = [nom[w][0] for w in wells]
ny = [nom[w][1] for w in wells]
ax.scatter(nx, ny, s=8, c="#1f77b4", label="JSON nominal", zorder=3)

# model points + exaggerated error vectors
for w in wells:
    if w not in model_xy:
        continue
    nx0, ny0 = nom[w]
    ex, ey = (model_xy[w] - nom[w]) * ERR_GAIN
    is_taught = status[w] == "taught"
    ax.plot([nx0, nx0 + ex], [ny0, ny0 + ey],
            c="#d62728" if is_taught else "#999999", lw=0.8, zorder=2)
    ax.scatter([nx0 + ex], [ny0 + ey], s=18,
               marker="x" if is_taught else "+",
               c="#d62728" if is_taught else "#7f7f7f", zorder=4)
    # call out the gross outliers with their true (un-exaggerated) error
    if is_taught and err.get(w, 0) > 3.0:
        ax.annotate(f"{w}: {err[w]:.1f} mm", (nx0, ny0),
                    textcoords="offset points", xytext=(6, 6),
                    fontsize=8, color="#d62728", weight="bold")

# legend proxies
ax.scatter([], [], marker="x", c="#d62728", label=f"model @ taught (n={len(taught_err)})")
ax.scatter([], [], marker="+", c="#7f7f7f", label="model @ untaught (F/G etc.)")

ax.set_aspect("equal")
ax.set_xlabel("plate-local X (mm)")
ax.set_ylabel("plate-local Y (mm)")
ax.set_title(f"Current 5-bar model vs nominal 96-well grid  "
             f"({plate.load_name})\n"
             f"taught RMS = {rms:.2f} mm, max = {mx:.2f} mm  "
             f"| error vectors exaggerated x{ERR_GAIN:.0f}")
ax.legend(loc="upper right", framealpha=0.9)
ax.grid(True, ls=":", alpha=0.3)
# frame the plate (+ margin) so a single gross outlier vector can't dominate scale
xs = [nom[w][0] for w in wells]
ys = [nom[w][1] for w in wells]
ax.set_xlim(min(xs) - 12, max(xs) + 12)
ax.set_ylim(min(ys) - 12, max(ys) + 12)
fig.tight_layout()

out = "model_vs_json_overlay.png"
fig.savefig(out, dpi=130)
print(f"saved {out}")
print(f"plate: {plate.load_name}  wells: {len(wells)}")
print(f"model: {model.summary()}")
print(f"taught wells reconstructed: {len(taught_err)}  RMS {rms:.2f} mm  max {mx:.2f} mm")
