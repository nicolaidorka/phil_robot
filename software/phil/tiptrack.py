"""Live converged-tip tracker + interactive teach for the Phil 5-bar arm.

Opens a window of the well-plate grid with a marker that follows the 5-bar
*converged outlet (tip)* in real time as you jog the arm -- so positioning is
observable instead of blind. The tip is ``KinematicModel.forward(jX, jY)``: the
intersection of the two distal links, in plate-local mm (the same frame the grid
is drawn in, so no transform is needed).

It is GUIDED: an on-screen coach (right-hand panel) always tells you the next
step. The simple path is jog -> Enter -> fit:

    cd software
    python3 -m phil.tiptrack --simulate          # no hardware (demo, never writes real config)
    python3 -m phil.tiptrack --backend v2         # real hardware
    python3 -m phil.tiptrack --backend v2 --advanced   # + click-to-log + grid fit

Keys (hover the window so it has focus; tap arrows -- one move per press):
    arrows      jog X (up/down) / Y (left/right)
    PgUp/PgDn   jog Z up / down
    [ / ]       smaller / bigger jog step
    Enter       lock the well the orange ring is on (teach its joints)
    f           re-fit the 5-bar model from the taught wells (fitkin)
    q           quit (auto-fitkin if you taught anything this session)

--advanced adds: LEFT-CLICK where the outlet REALLY is to log a joints<->true-mm
sample (phil_tip_samples.jsonl), and ``g`` to fit the grid from those points.

The on-disk kinematics is v2-scale, so the tip is meaningful only on the v2
backend or in simulation; on legacy/stock it refuses (counts ~32x off the model).
Under --simulate, teaching and fitkin write to a throwaway temp dir, never the
real phil_teach.json / phil_kinematics.json.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import tempfile
import time
from collections import deque

import numpy as np

from .geometry import kinematics as _kin
from .geometry.kinematics import KinematicModel
from .paths import DEFAULT_TEACH_PATH
from .robot import PhilRobot
from .stepcheck import neighbour_score

# jog step ladders (repo usteps). v2 microstep model -> larger counts per mm.
STEPS_V2 = [64, 128, 256, 512, 1024]      # ~0.5/1/2/4/8 mm per press
STEPS_LEGACY = [8, 16, 32, 64, 120]
DEFAULT_STEP_IDX = 2


def _resolve_model(bot):
    """The fitted v2 model to compute the tip with, or None."""
    if KinematicModel is None:
        return None
    model = bot.kin_model or KinematicModel.load()
    return model if (model and model.is_fitted) else None


def main(argv=None):
    raw = list(argv if argv is not None else sys.argv[1:])
    ap = argparse.ArgumentParser(prog="phil.tiptrack",
                                 description="Live 5-bar tip tracker + interactive teach.")
    from .constants import DEFAULT_BACKEND
    ap.add_argument("--simulate", action="store_true", help="no hardware")
    ap.add_argument("--backend", default=DEFAULT_BACKEND,
                    choices=["legacy", "v2", "stock", "sim"])
    ap.add_argument("--labware", default=None)
    ap.add_argument("--trail", type=int, default=40, help="trail length (points)")
    ap.add_argument("--interval", type=int, default=100, help="refresh interval (ms)")
    ap.add_argument("--starts", type=int, default=80, help="fitkin multistart count")
    ap.add_argument("--advanced", action="store_true",
                    help="enable click-to-log real tip + G (fit grid from points)")
    ap.add_argument("--free", action="store_true",
                    help="free mode (jog + Enter locks nearest well); default is the guided wizard")
    ap.add_argument("--points", default=None,
                    help="comma-separated wizard targets (default: A1,A12,H1,H12,C4,F9)")
    args = ap.parse_args(raw)

    backend = "sim" if args.simulate else args.backend
    bot = PhilRobot(backend=backend, simulate=args.simulate, labware_path=args.labware)
    bot.connect()

    try:
        model = _resolve_model(bot)
        if model is None:
            print("no fitted kinematics model (phil_kinematics.json) -- run fitkin first.")
            return 1
        # the model is v2-scale; legacy/stock report counts ~32x off -> the tip
        # would barely move and sit in a corner. Refuse rather than mislead.
        if backend in ("legacy", "stock") and model.looks_v2():
            print("kinematics is v2-scale; run with --backend v2 or --simulate.")
            return 1
        bot.kin_model = model
        steps = STEPS_V2 if model.looks_v2() else STEPS_LEGACY

        if args.simulate:
            # seed the sim joint counter near the taught frame so the tip starts
            # over the plate (sim starts at 0,0 which is off the edge), stop jogs
            # from overwriting the real power-cycle frame, and redirect ALL config
            # writes (teach + fitkin) to a throwaway dir so the demo is harmless.
            if model.j_ref is not None:
                bot.mc.x_pos = int(round(model.j_ref[0]))
                bot.mc.y_pos = int(round(model.j_ref[1]))
            bot._save_frame = lambda *a, **k: None
            tmp = tempfile.mkdtemp(prefix="phil_tiptrack_sim_")
            bot.teach_path = os.path.join(tmp, "phil_teach.json")
            _kin.DEFAULT_KIN_PATH = os.path.join(tmp, "phil_kinematics.json")
            print(f"[simulate] teach/fitkin will write to {tmp} (real config untouched)")

        if args.free:
            _run_gui(bot, model, bot.plate, steps, args)
        else:
            _run_wizard(bot, model, bot.plate, steps, args)
        return 0
    finally:
        bot.close()


def _build_figure(bot, plate, advanced):
    """Plate axes on the left; a stack of clip-boxed sub-axes on the right (one per
    band) so text can never spill between bands or onto the plate. Returns the figure
    and a dict of artists/axes the GUI updates."""
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from matplotlib.lines import Line2D
    from .viz import draw_plate_grid

    for k in ("keymap.save", "keymap.pan", "keymap.grid", "keymap.quit",
              "keymap.back", "keymap.forward", "keymap.fullscreen", "keymap.yscale"):
        if k in plt.rcParams:
            plt.rcParams[k] = []
    plt.rcParams["toolbar"] = "None"

    fig = plt.figure(figsize=(15, 9))
    gs = fig.add_gridspec(1, 2, width_ratios=[3.0, 1.25], wspace=0.05,
                          left=0.06, right=0.985, top=0.93, bottom=0.05)
    ax = fig.add_subplot(gs[0, 0])
    side = gs[0, 1].subgridspec(5, 1, height_ratios=[3, 2, 3, 4, 2], hspace=0.5)
    ax_coach, ax_prog, ax_read, ax_legend, ax_keys = (fig.add_subplot(side[i]) for i in range(5))
    for a in (ax_coach, ax_prog, ax_read, ax_legend, ax_keys):
        a.axis("off")

    # --- plate (no text drawn here -- only markers) ---
    draw_plate_grid(ax, plate)
    ax.set_title(f"Phil teach -- {plate.load_name} ({bot.backend})", fontsize=12)
    taught_sc = ax.scatter([], [], s=120, marker="s", facecolors="none",
                           edgecolors="#1a7d1a", linewidths=1.8, zorder=4)
    target_ring = ax.scatter([], [], s=320, marker="o", facecolors="none",
                             edgecolors="#ff8c00", linewidths=2.2, zorder=7)
    clicked_sc = ax.scatter([], [], s=90, marker="x", c="#d62728",
                            linewidths=2.0, zorder=9)
    err_lc = LineCollection([], colors="#d62728", linestyles=(0, (4, 2)),
                            linewidths=1.2, zorder=7)
    ax.add_collection(err_lc)
    trail = LineCollection([], zorder=5)
    ax.add_collection(trail)
    tip, = ax.plot([], [], "o", ms=14, mec="k", mfc="#2ca02c", zorder=8)

    # --- side bands (one artist each, anchored top-left in its own clip box) ---
    coach = ax_coach.text(0.04, 0.96, "", va="top", ha="left", fontsize=11,
                          transform=ax_coach.transAxes, wrap=False,
                          bbox=dict(boxstyle="round", fc="#eef6ff", ec="#9bf", pad=0.6))
    prog = ax_prog.text(0.04, 0.96, "", va="top", ha="left", fontsize=9.5,
                        transform=ax_prog.transAxes, family="monospace")
    read = ax_read.text(0.04, 0.96, "", va="top", ha="left", fontsize=9,
                        transform=ax_read.transAxes, family="monospace")

    mk = lambda **kw: Line2D([], [], ls="none", **kw)
    handles = [
        mk(marker="o", mfc="none", mec="#1f77b4", ms=10, label="well"),
        mk(marker="o", mfc="#2ca02c", mec="k", ms=10, label="live tip (outlet)"),
        mk(marker="o", mfc="none", mec="#ff8c00", mew=2, ms=11, label="Enter locks this"),
        mk(marker="s", mfc="none", mec="#1a7d1a", mew=1.8, ms=10, label="taught well"),
    ]
    if advanced:
        handles += [
            mk(marker="x", mec="#d62728", mew=2, ms=10, label="logged real tip"),
            Line2D([], [], color="#d62728", lw=1.4, ls=(0, (4, 2)), label="model->real error"),
        ]
    ax_legend.legend(handles=handles, loc="center", fontsize=8, frameon=True,
                     framealpha=1.0, labelspacing=0.6, borderpad=0.5, handletextpad=0.6)

    if advanced:
        keys_txt = ("KEYS\n"
                    "arrows  jog X/Y\n"
                    "PgUp/Dn jog Z\n"
                    "[  ]    step size\n"
                    "Enter   lock well\n"
                    "F       fit model\n"
                    "click   log real tip\n"
                    "G       fit from points\n"
                    "Q       quit")
    else:
        keys_txt = ("KEYS\n"
                    "arrows  jog X/Y\n"
                    "PgUp/Dn jog Z\n"
                    "[  ]    step size\n"
                    "Enter   lock well\n"
                    "F       fit model\n"
                    "Q       quit")
    keys = ax_keys.text(0.04, 0.96, keys_txt, va="top", ha="left", fontsize=8.5,
                        transform=ax_keys.transAxes, family="monospace")

    return fig, dict(ax=ax, taught_sc=taught_sc, target_ring=target_ring,
                     clicked_sc=clicked_sc, err_lc=err_lc, trail=trail, tip=tip,
                     coach=coach, prog=prog, read=read, keys=keys)


def _usteps_per_mm(model):
    """Approx motor usteps per mm near the taught frame (X-axis proxy), or None."""
    if model.j_ref is None:
        return None
    e0 = model.forward(model.j_ref[0], model.j_ref[1])
    e1 = model.forward(model.j_ref[0] + 100, model.j_ref[1])
    if e0 is None or e1 is None:
        return None
    d = float(np.hypot(*(e1 - e0)))
    return d / 100.0 if d > 1e-9 else None


def _run_gui(bot, model, plate, steps, args):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from .geometry.teach import plate_corners

    fig, art = _build_figure(bot, plate, args.advanced)
    ax = art["ax"]
    upm = _usteps_per_mm(model)            # usteps per mm (None -> show usteps only)
    corners = plate_corners(plate)
    samples_path = os.path.join(
        os.path.dirname(bot.teach_path or DEFAULT_TEACH_PATH), "phil_tip_samples.jsonl")

    st = {"step_idx": DEFAULT_STEP_IDX, "in_jog": False,
          "pts": deque(maxlen=max(2, args.trail)), "session": set(),
          "clicks": [], "click_pts": [], "err_segs": [],
          "fitted": False, "last_lock": None, "read_cache": None}

    def step_label():
        s = steps[st["step_idx"]]
        return f"{s} ({s / upm:.1f}mm)" if upm else f"{s} usteps"

    def refresh_taught():
        offs = [plate.local_xy(w) for w in bot.teach_table.taught if w in plate]
        art["taught_sc"].set_offsets(np.array(offs) if offs else np.empty((0, 2)))

    # ---------------------------------------------------------------- coach
    def coach_text():
        n = len(bot.teach_table.taught)
        done = [c for c in corners if bot.teach_table.is_taught(c)]
        todo = [c for c in corners if not bot.teach_table.is_taught(c)]
        ll = st["last_lock"]
        if ll and ll.get("off"):
            return ("#fff0f0", "#c44",
                    f"[!] {ll['well']} looks OFF-GRID ({ll['mm']:.1f} mm)\n"
                    f"vs its neighbours. Re-center the\noutlet on it and press Enter again.")
        if st["fitted"]:
            return ("#eefbe9", "#5a5",
                    f"[ok] FITTED  (RMS {st['fitted']:.2f} mm)\n"
                    f"The arm can reach any well now.\nLock more to refine, or press Q.")
        if n < 4:
            nxt = todo[0] if todo else "any well"
            return ("#eef6ff", "#9bf",
                    f"STEP 1 of 2 - LOCK WELLS\n"
                    f"Jog the outlet onto {nxt}'s\ncenter, then press Enter.\n"
                    f"(locked {n}; need >=4, corners first)")
        return ("#eef6ff", "#9bf",
                f"STEP 2 of 2 - FIT\n"
                f"{n} wells locked. Press F to fit\nthe grid (or lock a few more).")

    def refresh_coach():
        fc, ec, msg = coach_text()
        art["coach"].set_text(msg)
        art["coach"].get_bbox_patch().set(facecolor=fc, edgecolor=ec)

    def refresh_prog():
        marks = "  ".join(f"{c}[{'v' if bot.teach_table.is_taught(c) else 'x'}]"
                          for c in corners)
        art["prog"].set_text(f"PROGRESS\ntaught wells: {len(bot.teach_table.taught)}\n"
                             f"corners: {marks}")

    def set_status(msg, _c=None):     # transient line shown in the readout band
        st["read_cache"] = None
        art["read"].set_text(msg)
        fig.canvas.draw_idle(); fig.canvas.flush_events()

    refresh_taught(); refresh_coach(); refresh_prog()

    # ---------------------------------------------------------------- live loop
    def update(_frame):
        j = bot.joint_position()
        E = model.forward(j["X"], j["Y"])
        if E is None:
            art["tip"].set_data([], [])
            art["target_ring"].set_offsets(np.empty((0, 2)))
            txt = (f"READOUT\nOUT OF REACH -- jog back\n"
                   f"X={j['X']:+d} Y={j['Y']:+d} Z={j['Z']:+d}\nstep {step_label()}")
        else:
            x, y = float(E[0]), float(E[1])
            art["tip"].set_data([x], [y])
            st["pts"].append((x, y))
            pts = list(st["pts"])
            if len(pts) >= 2:
                segs = [[pts[i], pts[i + 1]] for i in range(len(pts) - 1)]
                a = np.linspace(0.05, 0.85, len(segs))
                art["trail"].set_segments(segs)
                art["trail"].set_color([(0.17, 0.63, 0.17, ai) for ai in a])
            well, dist = plate.nearest_well(x, y)
            art["target_ring"].set_offsets(np.array([plate.local_xy(well)]))
            txt = (f"READOUT\ntip  x={x:6.1f}  y={y:6.1f} mm\n"
                   f"Enter locks: {well} ({dist:.1f} mm)\n"
                   f"X={j['X']:+d} Y={j['Y']:+d} Z={j['Z']:+d}\nstep {step_label()}")
        if txt != st["read_cache"]:        # only repaint text when it changed (no flicker)
            st["read_cache"] = txt
            art["read"].set_text(txt)

    # ---------------------------------------------------------------- actions
    def do_teach():
        j = bot.joint_position()
        E = model.forward(j["X"], j["Y"])
        if E is None:
            set_status("READOUT\ncan't lock: out of reach"); return
        well, dist = plate.nearest_well(float(E[0]), float(E[1]))
        bot.teach_well(well)
        bot.teach_table.save(bot.teach_path)
        st["session"].add(well)
        st["fitted"] = False
        ns = neighbour_score(plate, bot.teach_table, model, well)
        off = ns["n_neighbours"] > 0 and ns["flag"]
        st["last_lock"] = {"well": well, "mm": ns["score_mm"], "off": off}
        refresh_taught(); refresh_prog(); refresh_coach()

    def do_fitkin():
        if len(bot.teach_table.taught) < 4:
            set_status("READOUT\nneed >=4 taught wells to fit"); return
        set_status("READOUT\nfitting 5-bar model...")
        try:
            rms = bot.fit_kinematics(n_starts=args.starts)
        except Exception as e:
            set_status(f"READOUT\nfit failed: {e}"); return
        model.__dict__.update(bot.kin_model.__dict__)
        st["pts"].clear(); st["fitted"] = rms; st["last_lock"] = None
        refresh_coach()

    def do_click(event):
        if event.inaxes is not ax or event.button != 1 or event.xdata is None:
            return
        cx, cy = float(event.xdata), float(event.ydata)
        j = bot.joint_position()
        E = model.forward(j["X"], j["Y"])
        pred = None if E is None else [float(E[0]), float(E[1])]
        err = None if pred is None else float(np.hypot(pred[0] - cx, pred[1] - cy))
        well, dist = plate.nearest_well(cx, cy)
        rec = {"t": time.time(), "backend": bot.backend,
               "ustep_scale": getattr(model, "ustep_scale", None),
               "joints": {"X": j["X"], "Y": j["Y"], "Z": j["Z"]},
               "model_tip_mm": pred, "true_tip_mm": [cx, cy], "error_mm": err,
               "nearest_well": well, "dist_to_well_mm": dist, "step": steps[st["step_idx"]]}
        try:
            with open(samples_path, "a") as fh:
                fh.write(json.dumps(rec) + "\n")
        except OSError as e:
            set_status(f"READOUT\nsample log failed: {e}"); return
        st["clicks"].append(rec); st["click_pts"].append((cx, cy))
        art["clicked_sc"].set_offsets(np.array(st["click_pts"]))
        if pred is not None:
            st["err_segs"].append([pred, [cx, cy]])
            art["err_lc"].set_segments(st["err_segs"])
        set_status(f"READOUT\nlogged real tip near {well}\n"
                   + ("model out of reach" if err is None else f"model error {err:.1f} mm")
                   + f"\nsamples: {len(st['clicks'])}  (G = fit)")

    def do_gridfit():
        extra = [((s["true_tip_mm"][0], s["true_tip_mm"][1]),
                  (s["joints"]["X"], s["joints"]["Y"]), s["joints"]["Z"]) for s in st["clicks"]]
        if len(extra) + len(bot.teach_table.taught) < 4:
            set_status("READOUT\nneed >=4 points (click more / lock wells)"); return
        set_status("READOUT\nfitting grid from points...")
        try:
            m = KinematicModel()
            rms = m.fit(plate, bot.teach_table, n_starts=args.starts, extra_points=extra)
            m.save()
        except Exception as e:
            set_status(f"READOUT\ngrid fit failed: {e}"); return
        bot.kin_model = m; model.__dict__.update(m.__dict__)
        st["pts"].clear(); st["fitted"] = rms; st["last_lock"] = None
        refresh_coach()

    # ---------------------------------------------------------------- input
    def on_key(event):
        if event.key == "q":
            if st["session"] and not st["fitted"]:
                try:
                    do_fitkin()
                except Exception:
                    pass
            plt.close(fig); return
        if event.key in ("enter", "return"):
            do_teach(); return
        if event.key == "f":
            do_fitkin(); return
        if event.key == "g" and args.advanced:
            do_gridfit(); return
        if event.key in ("[", "-", "_"):
            st["step_idx"] = max(0, st["step_idx"] - 1); st["read_cache"] = None; return
        if event.key in ("]", "+", "="):
            st["step_idx"] = min(len(steps) - 1, st["step_idx"] + 1); st["read_cache"] = None; return
        if st["in_jog"]:
            return
        step = steps[st["step_idx"]]
        delta = {"up": dict(dx=step), "down": dict(dx=-step),
                 "right": dict(dy=step), "left": dict(dy=-step),
                 "pageup": dict(dz=step), "pagedown": dict(dz=-step)}.get(event.key)
        if not delta:
            return
        st["in_jog"] = True
        try:
            bot.jog_joint(**delta)
        finally:
            st["in_jog"] = False

    def _grab_focus(_evt=None):
        try:
            fig.canvas.setFocus()
        except Exception:
            pass

    def _grab_focus_once(_evt):
        if not st.get("focused"):
            st["focused"] = True
            _grab_focus()

    fig.canvas.mpl_connect("key_press_event", on_key)
    if args.advanced:
        fig.canvas.mpl_connect("button_press_event", do_click)
    fig.canvas.mpl_connect("figure_enter_event", _grab_focus)
    fig.canvas.mpl_connect("draw_event", _grab_focus_once)
    fig.canvas.mpl_connect("resize_event", lambda e: (refresh_coach(), refresh_prog()))
    anim = FuncAnimation(fig, update, interval=args.interval, blit=False,
                         cache_frame_data=False)
    fig._tiptrack_anim = anim
    plt.show()


DEFAULT_WIZARD_POINTS = ["A1", "A12", "H1", "H12", "C4", "F9"]


def _run_wizard(bot, model, plate, steps, args):
    """Guided few-point calibration: set travel height, then for each target the arm
    travels (lift + XY approach, no descend), you lower/center and Enter to lock; fit
    at the end from only this run's points (old table backed up). One consistent frame
    (no re-zero). See the plan for the safety rationale."""
    import shutil
    import traceback
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    _cfgdir = os.path.dirname(bot.teach_path or DEFAULT_TEACH_PATH)
    logpath = os.path.join(_cfgdir, "phil_tiptrack.log")
    locks_path = os.path.join(_cfgdir, "phil_wizard_locks.jsonl")

    def log(msg):
        try:
            with open(logpath, "a") as f:
                f.write(msg + "\n")
        except OSError:
            pass

    if args.points and args.points.strip().lower() == "all":
        # every well, snake order (row 0 L->R, row 1 R->L, ...) so each hop is one pitch
        targets = []
        for ri, r in enumerate(plate.rows):
            cols = plate.columns if ri % 2 == 0 else list(reversed(plate.columns))
            targets += [f"{r}{c}" for c in cols]
    elif args.points:
        targets = [w.strip().upper() for w in args.points.split(",") if w.strip()]
    else:
        targets = list(DEFAULT_WIZARD_POINTS)
    targets = [w for w in targets if w in plate]
    if not targets:
        print("no valid wizard targets for this labware"); return

    fig, art = _build_figure(bot, plate, args.advanced)
    ax = art["ax"]
    upm = [_usteps_per_mm(model)]                  # mutable so we can recompute after fit
    samples_path = os.path.join(
        os.path.dirname(bot.teach_path or DEFAULT_TEACH_PATH), "phil_tip_samples.jsonl")

    art["keys"].set_text("KEYS\narrows  jog X/Y\nPgUp/Dn jog Z\n-  +    step down/up\n"
                         "T       set travel height\nEnter   lock this well\n"
                         "B       go back\nF       fit (at end)\nQ       quit")

    st = {"step_idx": DEFAULT_STEP_IDX, "in_jog": False,
          "pts": deque(maxlen=max(2, args.trail)),
          "phase": "set_travel", "idx": 0, "session": set(),
          "confirm": None, "fit_rms": None, "read_cache": None}

    def step_label():
        s = steps[st["step_idx"]]
        return f"{s} ({s / upm[0]:.1f}mm)" if upm[0] else f"{s} usteps"

    def cur_target():
        return targets[st["idx"]] if st["phase"] in ("run",) and st["idx"] < len(targets) else None

    # ----------------------------------------------------------- side panels
    def set_status(msg):
        st["read_cache"] = None
        art["read"].set_text(msg)
        fig.canvas.draw_idle(); fig.canvas.flush_events()

    def refresh_prog():
        cells = []
        for i, w in enumerate(targets):
            mark = "v" if w in st["session"] else (">" if i == st["idx"] and st["phase"] == "run" else " ")
            cells.append(f"{w}[{mark}]")
        # wrap 3 per line so it fits the panel
        lines = ["  ".join(cells[i:i + 3]) for i in range(0, len(cells), 3)]
        art["prog"].set_text("PROGRESS  ({}/{})\n".format(len(st["session"]), len(targets))
                             + "\n".join(lines))

    def refresh_coach():
        tgt = cur_target()
        if st["phase"] == "set_travel":
            fc, ec, msg = ("#fff7e6", "#e9b", "SETUP\nRaise the nozzle to a safe\n"
                           "height above the plate (PgUp),\nthen press T.")
        elif st["phase"] == "run" and tgt == targets[0]:
            fc, ec, msg = ("#eef6ff", "#9bf", f"STEP 1/{len(targets)} - ANCHOR {tgt}\n"
                           f"Lower (PgDn) onto {tgt}'s center,\nthen Enter. This is your anchor.")
        elif st["phase"] == "run":
            fc, ec, msg = ("#eef6ff", "#9bf",
                           f"STEP {st['idx']+1}/{len(targets)} - {tgt}\n"
                           f"The arm drove here. Lower & nudge\nto center, then Enter.  (B = back)")
        elif st["phase"] == "done":
            fc, ec, msg = ("#eefbe9", "#5a5", f"ALL {len(st['session'])} POINTS SET\n"
                           "Press F to fit the grid\nfrom your points.  (B = back)")
        elif st["phase"] == "learning":
            fc, ec, msg = ("#fff7e6", "#e9b", "LEARNING the grid from\nyour points... (window\n"
                           "pauses a few seconds)")
        else:  # fitted
            fc, ec, msg = ("#eefbe9", "#5a5", f"[ok] FITTED  RMS {st['fit_rms']:.2f} mm\n"
                           "The model now follows your\npoints. Press Q to finish.")
        if st["confirm"]:
            fc, ec, msg = ("#fff0f0", "#c44", st["confirm"])
        art["coach"].set_text(msg)
        art["coach"].get_bbox_patch().set(facecolor=fc, edgecolor=ec)

    def refresh_taught():
        offs = [plate.local_xy(w) for w in st["session"] if w in plate]
        art["taught_sc"].set_offsets(np.array(offs) if offs else np.empty((0, 2)))

    # ----------------------------------------------------------- travel
    def target_joints(well):
        if bot.teach_table.is_taught(well):
            d = bot.teach_table.taught[well]
            return d["X"], d["Y"]
        try:
            d = bot.kin_model.predict(well, plate)
            return d["X"], d["Y"]
        except Exception:
            return None

    def travel_to(well):
        tz = bot.teach_table.z_travel_usteps
        jt = target_joints(well)
        if tz is None or jt is None:
            return False
        set_status(f"READOUT\ntraveling to {well}...")
        try:
            bot._move_joints_to(z=tz)            # lift to safe height
            bot._approach_joints(jt[0], jt[1])   # XY only, clamped; NO descend
        except Exception as e:
            set_status(f"READOUT\ntravel failed: {e}")
            return False
        return True

    # ----------------------------------------------------------- live loop
    def update(_frame):
        j = bot.joint_position()
        E = model.forward(j["X"], j["Y"])
        tgt = cur_target()
        if E is None:
            art["tip"].set_data([], [])
            art["target_ring"].set_offsets(np.empty((0, 2)))
            txt = (f"READOUT\nOUT OF REACH -- jog back\n"
                   f"X={j['X']:+d} Y={j['Y']:+d} Z={j['Z']:+d}\nstep {step_label()}")
        else:
            x, y = float(E[0]), float(E[1])
            art["tip"].set_data([x], [y])
            st["pts"].append((x, y))
            pts = list(st["pts"])
            if len(pts) >= 2:
                segs = [[pts[i], pts[i + 1]] for i in range(len(pts) - 1)]
                a = np.linspace(0.05, 0.85, len(segs))
                art["trail"].set_segments(segs)
                art["trail"].set_color([(0.17, 0.63, 0.17, ai) for ai in a])
            ring = tgt if tgt else plate.nearest_well(x, y)[0]
            art["target_ring"].set_offsets(np.array([plate.local_xy(ring)]))
            lockline = f"Enter locks: {tgt}" if tgt else "(free)"
            txt = (f"READOUT\ntip  x={x:6.1f}  y={y:6.1f} mm\n{lockline}\n"
                   f"X={j['X']:+d} Y={j['Y']:+d} Z={j['Z']:+d}\nstep {step_label()}")
        if txt != st["read_cache"]:
            st["read_cache"] = txt
            art["read"].set_text(txt)

    # ----------------------------------------------------------- actions
    def advance_after_lock():
        st["session"].add(targets[st["idx"]])
        if st["idx"] < len(targets) - 1:
            st["idx"] += 1
            refresh_prog(); refresh_coach(); refresh_taught()
            travel_to(targets[st["idx"]])
        else:
            st["phase"] = "done"
            refresh_prog(); refresh_coach(); refresh_taught()

    def do_lock():
        tgt = cur_target()
        if tgt is None:
            return
        # YOU placed the nozzle -> this IS the truth for tgt. No nearest-well second-
        # guessing (the model is exactly what we're calibrating, so its "nearest" lies).
        p = bot.teach_well(tgt)
        bot.teach_table.save(bot.teach_path)        # persist into the teach table now
        # AND append an immutable record of THIS lock the moment it happens (not at the
        # end) -- so a crash/kill never loses a point you locked.
        try:
            with open(locks_path, "a") as f:
                f.write(json.dumps({"t": time.time(), "well": tgt,
                                    "joints": {"X": p["X"], "Y": p["Y"], "Z": p["Z"]}}) + "\n")
        except OSError:
            pass
        log(f"locked {tgt} @ X={p['X']} Y={p['Y']} Z={p['Z']}")
        st["pts"].clear()
        advance_after_lock()

    def do_back():
        if st["phase"] == "done":
            st["phase"] = "run"
        elif st["phase"] != "run" or st["idx"] == 0:
            return                                # can't go back past the A1 anchor
        else:
            st["idx"] -= 1
        st["confirm"] = None
        st["session"].discard(targets[st["idx"]])
        refresh_prog(); refresh_coach(); refresh_taught()
        travel_to(targets[st["idx"]])

    def show_fit():
        segs, offs, cols, errs = [], [], [], []
        tol = 2.5
        for w in sorted(bot.teach_table.taught):
            d = bot.teach_table.taught[w]
            E = model.forward(d["X"], d["Y"])
            if E is None or w not in plate:
                continue
            nx, ny = plate.local_xy(w)
            er = float(np.hypot(float(E[0]) - nx, float(E[1]) - ny))
            errs.append((w, er)); segs.append([[float(E[0]), float(E[1])], [nx, ny]])
            offs.append((nx, ny))
            cols.append("#d62728" if er > tol else ("#e8a" if er > tol / 2 else "#1a7d1a"))
        art["err_lc"].set_segments(segs); art["err_lc"].set_color("#d62728")
        if offs:
            art["taught_sc"].set_offsets(np.array(offs))
            art["taught_sc"].set_edgecolors(cols)
        if errs:
            rms = (sum(e * e for _, e in errs) / len(errs)) ** 0.5
            worst = max(errs, key=lambda t: t[1])
            st["fit_rms"] = rms
            set_status(f"READOUT\nFIT vs JSON grid:\nRMS {rms:.2f} mm\n"
                       f"worst {worst[1]:.1f} mm at {worst[0]}")

    def do_fit():
        session = sorted(st["session"])
        if len(session) < 4:
            set_status(f"READOUT\nneed >=4 points to fit;\nhave {len(session)}. Lock more.")
            return
        # backup the on-disk teach table (sim-safe: source is bot.teach_path)
        src = bot.teach_path or DEFAULT_TEACH_PATH
        try:
            if os.path.isfile(src):
                shutil.copy(src, src[:-5] + f".backup-{int(time.time())}.json")
        except OSError:
            pass
        # Fit on ONLY this run's freshly-locked points (so stale/poisoned older wells
        # can't skew the fit) WITHOUT discarding them: build a throwaway table for the
        # fit; bot.teach_table keeps EVERY taught well and stays on disk. (A blanket
        # wipe-on-fit is what silently cut the table 26->6; never replace, only refit.)
        from .geometry.teach import TeachTable
        fit_table = TeachTable(labware=bot.teach_table.labware,
                               z_travel_usteps=bot.teach_table.z_travel_usteps,
                               ustep_scale=bot.teach_table.ustep_scale)
        fit_table.taught = {w: dict(bot.teach_table.taught[w]) for w in session
                            if w in bot.teach_table.taught}
        try:
            from .geometry.calibration import Calibration
            bot.calibration = Calibration.nominal(plate)
        except Exception:
            pass
        bot.well_map = None
        # LEARN visibly: run several fit attempts, repainting between each so the window
        # shows progress instead of looking stuck. Smaller per-attempt n_starts keeps each
        # blocking chunk short. Whole thing wrapped so it can never kill the GUI.
        st["phase"] = "learning"; refresh_coach()
        best = None
        attempts = 3
        seeds = np.random.default_rng().integers(0, 2 ** 31 - 1, size=attempts)
        per = max(6, min(args.starts // attempts, 15))
        for k, seed in enumerate(seeds):
            set_status(f"READOUT\nLEARNING the grid...\nattempt {k+1}/{attempts}"
                       + (f"\nbest RMS {best[0]:.2f} mm" if best else ""))
            try:
                m = KinematicModel()
                rms = m.fit(plate, fit_table, n_starts=per, seed=int(seed))
            except Exception:
                log(f"fit attempt {k+1} error:\n" + traceback.format_exc())
                continue
            if best is None or rms < best[0]:
                best = (rms, m)
            if best[0] <= 1.0:
                break
        if best is None:
            set_status("READOUT\nfit failed (see phil_tiptrack.log)")
            st["phase"] = "done"; refresh_coach(); return       # table untouched, nothing to restore
        rms, m = best
        try:
            m.save()
        except Exception:
            log("model save error:\n" + traceback.format_exc())
        # The teach table already holds every locked point (do_lock saved each one) plus
        # the previously-taught wells. Persist the FULL table (idempotent) -- never a
        # subset, so coverage can't silently shrink on a fit.
        try:
            bot.teach_table.save(bot.teach_path)
        except Exception:
            log("teach save error:\n" + traceback.format_exc())
        bot.kin_model = m
        model.__dict__.update(m.__dict__)
        upm[0] = _usteps_per_mm(model)
        st["fit_rms"] = rms
        st["phase"] = "fitted"
        log(f"fit done: RMS {rms:.3f} on {session}; teach table kept "
            f"{len(bot.teach_table.taught)} well(s)")
        show_fit(); refresh_coach()

    # ----------------------------------------------------------- input
    def on_key(event):
        if event.key == "q":
            if st["session"] and st["phase"] != "fitted":
                do_fit()
            plt.close(fig); return
        if event.key == "t" and st["phase"] == "set_travel":
            bot.set_travel_z()
            st["phase"] = "run"; st["idx"] = 0
            refresh_prog(); refresh_coach()
            travel_to(targets[0]); return
        if event.key in ("enter", "return"):
            if st["phase"] == "run":
                do_lock()
            elif st["phase"] in ("done",):
                do_fit()
            return
        if event.key in ("b", "backspace"):
            do_back(); return
        if event.key == "f":
            if st["phase"] in ("run", "done"):
                do_fit()
            return
        if event.key == "g" and args.advanced:
            return                                  # grid-fit-from-clicks not used in wizard
        if event.key in ("[", "-", "_"):
            st["step_idx"] = max(0, st["step_idx"] - 1); st["read_cache"] = None; return
        if event.key in ("]", "+", "="):
            st["step_idx"] = min(len(steps) - 1, st["step_idx"] + 1); st["read_cache"] = None; return
        if st["in_jog"]:
            return
        step = steps[st["step_idx"]]
        delta = {"up": dict(dx=step), "down": dict(dx=-step),
                 "right": dict(dy=step), "left": dict(dy=-step),
                 "pageup": dict(dz=step), "pagedown": dict(dz=-step)}.get(event.key)
        if not delta:
            return
        st["in_jog"] = True
        try:
            bot.jog_joint(**delta)
        finally:
            st["in_jog"] = False

    def _grab_focus(_evt=None):
        try:
            fig.canvas.setFocus()
        except Exception:
            pass

    def _grab_focus_once(_evt):
        if not st.get("focused"):
            st["focused"] = True
            _grab_focus()

    def _safe(fn):
        def wrapped(*a, **k):
            try:
                return fn(*a, **k)
            except Exception:
                log(f"{getattr(fn, '__name__', 'cb')} error:\n" + traceback.format_exc())
                try:
                    set_status("READOUT\nhit an error (logged to\nphil_tiptrack.log). "
                               "Q to quit if stuck.")
                except Exception:
                    pass
        return wrapped

    refresh_prog(); refresh_coach(); refresh_taught()
    fig.canvas.mpl_connect("key_press_event", _safe(on_key))
    fig.canvas.mpl_connect("figure_enter_event", _grab_focus)
    fig.canvas.mpl_connect("draw_event", _grab_focus_once)
    fig.canvas.mpl_connect("resize_event", _safe(lambda e: (refresh_coach(), refresh_prog())))
    anim = FuncAnimation(fig, _safe(update), interval=args.interval, blit=False,
                         cache_frame_data=False)
    fig._tiptrack_anim = anim
    plt.show()


if __name__ == "__main__":
    sys.exit(main())
