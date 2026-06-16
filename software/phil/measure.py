"""Interactive offset measurement: drive each well via the MODEL, you jog the nozzle
to the TRUE centre, and it records how far (and which way) the model was off. This
turns "it's a bit off" into NUMBERS so we can see if the error is:

  * SYSTEMATIC (a constant offset/direction on every well) -> one frame correction
    (reanchor / a single shift) fixes nearly all of it; or
  * PER-WELL / random -> genuine model or teaching error -> teach those spots.

Run it in YOUR terminal (needs live keys):

    cd software
    python3 -m phil.measure --v2                 # default diagnostic spread of wells
    python3 -m phil.measure --v2 D9 E6 H6 B11     # specific wells
    python3 -m phil.measure --v2 --apply ...      # ALSO re-teach each centred well (exact)

Keys: arrows = jog X/Y, a/z = Z up/down, +/- = step size, Enter = record offset,
n = skip this well, q = finish. Saves offsets to config/phil_offsets.json.
"""
from __future__ import annotations

import json
import os
import sys
import termios
import tty

import numpy as np

from .robot import PhilRobot
from .constants import DEFAULT_BACKEND, CONTROLLER_SN

# Default diagnostic set: a 2-D spread of mostly-UNTAUGHT interior + edge wells (model
# error shows here) plus a couple of taught refs (A1/D6 -> backlash baseline).
DEFAULT_WELLS = ["A1", "D6", "E6", "C7", "F4", "D9", "G8", "B11", "F11", "H6", "C10", "H12"]


def _read_key():
    ch = sys.stdin.read(1)
    if ch == "\x1b":
        seq = sys.stdin.read(2)
        return {"[A": "UP", "[B": "DOWN", "[C": "RIGHT", "[D": "LEFT"}.get(seq, "ESC")
    if ch in ("\r", "\n"):
        return "ENTER"
    return ch


def _report(bot, results):
    if not results:
        print("\nno measurements recorded.")
        return
    kin = bot.kin_model if (bot.kin_model and bot.kin_model.is_fitted) else None
    print("\n=== OFFSETS  (model target -> where YOU centred it) ===")
    print(f"  {'well':4s} {'dX':>7s} {'dY':>7s} {'mm':>6s} {'dir(deg)':>9s}  {'src':>10s}")
    offs = []
    for r in results:
        dx, dy = r["offset"]
        offs.append((dx, dy))
        mm = dirn = ""
        if kin is not None:
            J = kin._jacobian(r["center"][0], r["center"][1])
            if J is not None:
                v = J @ np.array([dx, dy], float)       # joint offset -> plate mm
                mm = f"{float(np.hypot(*v)):.2f}"
                dirn = f"{float(np.degrees(np.arctan2(v[1], v[0]))):+.0f}"
        print(f"  {r['well']:4s} {dx:7d} {dy:7d} {mm:>6s} {dirn:>9s}  {r.get('src',''):>10s}")
    arr = np.array(offs, float)
    mean = arr.mean(0)
    # spread about the mean = the per-well component left after a single shift
    resid = arr - mean
    spread = np.sqrt((resid ** 2).sum(1)).mean()
    meanmag = float(np.hypot(*mean))
    print(f"\n  SYSTEMATIC mean offset: dX={mean[0]:+.0f} dY={mean[1]:+.0f} usteps "
          f"(~{meanmag/190:.2f} mm)")
    print(f"  PER-WELL spread (after removing the mean): ~{spread:.0f} usteps "
          f"(~{spread/190:.2f} mm)")
    if meanmag > 1.5 * spread and meanmag > 200:
        print("  => DOMINANTLY SYSTEMATIC: a single frame shift / reanchor removes most of it.")
    elif spread > meanmag:
        print("  => DOMINANTLY PER-WELL: model/teach error -> teach those wells (or refit).")
    else:
        print("  => MIXED: apply the systematic shift, then teach the few worst wells.")


def main(argv=None):
    raw = list(argv if argv is not None else sys.argv[1:])
    if "--legacy" in raw:
        use_v2 = False
    elif "--v2" in raw:
        use_v2 = True
    else:
        use_v2 = (DEFAULT_BACKEND == "v2")
    apply_teach = "--apply" in raw
    raw = [a for a in raw if a not in ("--v2", "--legacy", "--apply")]
    wells = [w.upper() for w in raw] or DEFAULT_WELLS

    bot = PhilRobot(backend="v2" if use_v2 else "legacy", controller_sn=CONTROLLER_SN)
    steps = [8, 16, 32, 64, 128, 256, 512, 1024] if use_v2 else [8, 16, 32, 64, 120]
    si = 3
    if use_v2:
        bot.teach_table.ustep_scale = 256
    bot.connect()
    if not (bot.kin_model and bot.kin_model.is_fitted):
        print("** no fitted kinematics model loaded -- measuring against the fallback map. "
              "Run fitkin first for a meaningful model test.")
    print(__doc__)
    print(f"measuring {len(wells)} wells: {wells}\n")

    tz = bot.teach_table.travel_z()
    if tz is None:                      # lift between wells so the nozzle never drags
        tz = 30000
        print("  (no travel-Z set; using a 30000-ustep lift between wells)")
    results = []
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        for well in wells:
            try:
                pred = bot.predict_well(well)            # MODEL prediction (kinematics)
                src = bot._resolve_well(well)[1]
            except Exception as e:
                print(f"  {well}: predict failed ({e}) -- skipping"); continue
            if tz is not None:
                bot._move_joints_to(z=tz)
            bot._approach_joints(pred["X"], pred["Y"])    # drive there like goto does
            if tz is not None:
                bot._move_joints_to(z=int(pred.get("Z", 0)))
            sys.stdout.write(
                f"\n=== {well} [{src}]: model put it at X={pred['X']} Y={pred['Y']}.\n"
                f"    Jog the nozzle to the TRUE centre of {well}.  "
                f"Enter=record  n=skip  q=finish\n")
            while True:
                j = bot.joint_position()
                sys.stdout.write(f"\r    step={steps[si]:4d}  joints X={j['X']:+6d} "
                                 f"Y={j['Y']:+6d} Z={j['Z']:+6d}        ")
                sys.stdout.flush()
                k = _read_key()
                moved = k in ("UP", "DOWN", "RIGHT", "LEFT", "a", "z")
                if k == "UP":
                    bot.jog_joint(dx=steps[si])
                elif k == "DOWN":
                    bot.jog_joint(dx=-steps[si])
                elif k == "RIGHT":
                    bot.jog_joint(dy=steps[si])
                elif k == "LEFT":
                    bot.jog_joint(dy=-steps[si])
                elif k == "a":
                    bot.jog_joint(dz=steps[si])
                elif k == "z":
                    bot.jog_joint(dz=-steps[si])
                elif k in ("+", "="):
                    si = min(si + 1, len(steps) - 1)
                elif k in ("-", "_"):
                    si = max(si - 1, 0)
                elif k == "ENTER":
                    cur = bot.joint_position()
                    off = (cur["X"] - pred["X"], cur["Y"] - pred["Y"])
                    results.append({"well": well, "src": src,
                                    "pred": [pred["X"], pred["Y"]],
                                    "center": [cur["X"], cur["Y"]],
                                    "offset": [off[0], off[1]]})
                    if apply_teach:
                        bot.teach_table.teach(well, cur["X"], cur["Y"], cur["Z"])
                    sys.stdout.write(f"\n    recorded {well}: offset dX={off[0]:+d} "
                                     f"dY={off[1]:+d} usteps\n")
                    break
                elif k == "n":
                    sys.stdout.write(f"\n    skipped {well}\n"); break
                elif k == "q":
                    raise KeyboardInterrupt
                if moved:
                    termios.tcflush(fd, termios.TCIFLUSH)   # drop keys typed during the move
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        if apply_teach and results:
            p = bot.teach_table.save(bot.teach_path)
            print(f"\nre-taught {len(results)} centred wells -> {p}")
        _report(bot, results)
        if results:
            from .paths import DEFAULT_TEACH_PATH
            base = bot.teach_path or DEFAULT_TEACH_PATH
            path = os.path.join(os.path.dirname(base), "phil_offsets.json")
            with open(path, "w") as f:
                json.dump(results, f, indent=2)
            print(f"\nsaved offsets -> {path}")
        bot.close()


if __name__ == "__main__":
    main()
