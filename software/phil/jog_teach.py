"""Real-time arrow-key jog + teach console for the Phil arm.

Run it in YOUR terminal (it needs live keyboard input):

    cd software
    python -m phil.jog_teach              # teach wells (arrow keys)
    python -m phil.jog_teach --all        # teach EVERY well (snake order, resumable)
    python -m phil.jog_teach --anchor     # anchor the 4 corners -> affine frame fix

Each arrow drives ONE motor by a clean whole step (no rounding):
    Up / Down       X motor +/-   (outlet ~ toward front-right / back-left)
    Right / Left    Y motor +/-   (outlet ~ toward front-left / back-right)
    a / z           raise / lower Z
    + / -           bigger / smaller jog step
    Enter           RECORD the current target well, advance to the next
    n               skip to the next target well (don't record)
    h               set HOME here (zero the joints at this pose) - do this on A1
    s               save now
    u               undo last recorded well
    q               save and quit

NOTE: the joints are rotary with some backlash, so reversing direction loses a
little motion. ``goto`` always closes in on a well from -X,-Y and finishes with a
+X,+Y creep, so teach each well THE SAME WAY: approach from the lower-left and make
your FINAL nudges Up and/or Right (don't finish on Down/Left). The console shows
"approach: +X+Y ok" when you're good; it still records either way but a Down/Left
finish lands a hair less precisely. The arm auto-approaches each well with that same
+X,+Y motion, so often you only fine-center and press Enter.

It tells you which well to go to. Jog the outlet over it, press Enter, repeat.
After a few wells it fits the mm<->joint map (from the labware JSON) so you can
`goto` any well; teach more if any land off.
"""
from __future__ import annotations

import sys
import termios
import time
import tty

from .robot import PhilRobot
from .geometry.well_plate import WellPlate

# Order I guide you through: 4 corners (best spread) + 2 middles to capture the
# arm's curve. You can change this list or just keep going past it.
GUIDE = ["A1", "A12", "H1", "H12", "D6", "E7"]


def _serpentine(plate):
    """All wells in a boustrophedon (snake) order: row A left->right, row B
    right->left, ... so the arm never flies across the plate between wells."""
    rc = {wid: WellPlate.parse_well_id(wid) for wid in plate.well_ids()}
    rows = sorted({r for r, _ in rc.values()})
    cols = sorted({c for _, c in rc.values()})
    byrc = {(r, c): wid for wid, (r, c) in rc.items()}
    order = []
    for i, r in enumerate(rows):
        seq = cols if i % 2 == 0 else list(reversed(cols))
        order += [byrc[(r, c)] for c in seq if (r, c) in byrc]
    return order


def _hint(target):
    h = HINTS.get(target)
    if h:
        return h
    try:
        r, c = WellPlate.parse_well_id(target)
        return f"row {chr(ord('A') + r)} ({r + 1} down), column {c + 1}"
    except Exception:
        return ""

# Plain-language hint for where each well is on the 8x12 grid (rows A..H,
# columns 1..12). A and 1 are one corner; H and 12 are the diagonal opposite.
HINTS = {
    "A1":  "row A, column 1  - the START corner (where you set home)",
    "A12": "row A, column 12 - SAME row as A1, all the way across to column 12",
    "H1":  "row H, column 1  - SAME column as A1, all the way down to row H",
    "H12": "row H, column 12 - the FAR corner, diagonally opposite A1",
    "D6":  "row D (4th down), column 6 - near the middle",
    "E7":  "row E (5th down), column 7 - near the middle",
}

# Jog sizes in repo usteps. The firmware moves in WHOLE full-steps (8 repo usteps
# each, ~1 mm of outlet travel near the plate), so these are multiples of 8 to
# avoid rounding to zero / uneven moves. Cycle with +/-.
STEPS = [8, 16, 32, 64, 120]        # = 1, 2, 4, 8, 15 full-steps (~1,2,4,8,15 mm)
DEFAULT_STEP_IDX = 1



def _read_key():
    """Read one keypress (arrow keys decoded) from a cbreak-mode stdin."""
    ch = sys.stdin.read(1)
    if ch == "\x1b":                 # escape sequence (arrow keys)
        seq = sys.stdin.read(2)
        return {"[A": "UP", "[B": "DOWN", "[C": "RIGHT", "[D": "LEFT"}.get(seq, "ESC")
    if ch in ("\r", "\n"):
        return "ENTER"
    return ch


def _finish_ok(last_dir):
    """The well must be reached with a +X,+Y (Up/Right) finish so goto, which
    always creeps +X,+Y, lands the outlet where it was taught. A reversal (the
    last X or Y nudge was Down/Left) leaves a ~1 backlash-gap offset."""
    return last_dir is None or (last_dir["X"] != -1 and last_dir["Y"] != -1)


def _status(bot, target, step, recorded, last_dir=None):
    j = bot.joint_position()
    tgt = target if target else "(free)"
    done = ",".join(recorded) if recorded else "none"
    flag = "+X+Y ok " if _finish_ok(last_dir) else "NUDGE Up/Right"
    sys.stdout.write(
        f"\r  TARGET: {tgt:4s} | X={j['X']:+5d} Y={j['Y']:+5d} Z={j['Z']:+5d}"
        f" | step={step:3d} | approach: {flag} | done: {done}        "
    )
    sys.stdout.flush()


def _announce(target):
    if target:
        sys.stdout.write(f"\n  >>> Go to {target}: {_hint(target)}\n")


def _approach(bot, target, anchor_mode=False, always=False):
    """Drive near the predicted spot so the user only nudges the last bit.

    Anchor mode (or ``always``, used by teach-all) uses the current corrected
    goto target, so even already-taught wells are auto-approached; plain teach
    mode uses the raw map and skips taught wells.
    """
    if not target:
        return
    try:
        if anchor_mode or always:
            p = bot._resolve_well(target)[0]          # current best estimate
        else:
            if bot.teach_table.is_taught(target):
                return
            if not (bot.well_map and bot.well_map.is_fitted):
                return
            p = bot.predict_well(target)
        sys.stdout.write(f"  (auto-approaching {target} -> "
                         f"X={p['X']} Y={p['Y']}; nudge then Enter)\n")
        # Use goto's coordinated +X,+Y approach so the well is reached the SAME way
        # goto will replay it: both arms close in together and settle in a +X,+Y
        # backlash state. The user then only fine-centers, finishing with Up/Right.
        bot._approach_joints(p["X"], p["Y"])
    except Exception as e:
        sys.stdout.write(f"  (auto-approach skipped: {e})\n")


def main(argv=None):
    import sys as _sys
    raw = list(argv if argv is not None else _sys.argv[1:])
    anchor_mode = "--anchor" in raw
    all_mode = "--all" in raw
    use_v2 = "--v2" in raw                      # post-reflash microstep firmware
    raw = [a for a in raw if a not in ("--anchor", "--all", "--v2")]
    # When specific wells are named (re-teaching a few), auto-drive to each one
    # like --all does, so you just fine-center instead of jogging there by hand.
    auto_approach = all_mode or (not anchor_mode and bool(raw))

    bot = PhilRobot(backend="v2" if use_v2 else "legacy")
    # On v2 the joint counts are 32x finer (microsteps, not full-steps), so scale
    # the jog increments to keep the same physical nudge sizes.
    if bot._ustep_scale != 1:
        global STEPS
        STEPS = [s * bot._ustep_scale for s in STEPS]
    bot.connect()

    if anchor_mode:
        # Capture the 4 corners as an affine frame correction (no teaching, no refit).
        bot.clear_anchors()
        guide = list(bot.ANCHOR_WELLS)
    elif all_mode:
        # Teach EVERY well on the plate, in a snake order to minimise travel.
        guide = _serpentine(bot.plate)
    else:
        # Optional: pass specific wells to teach/refine, e.g.
        #   python -m phil.jog_teach A6 H6 C1 C12
        guide = [w.upper() for w in raw] or list(GUIDE)

    print(__doc__)
    label = "ANCHOR" if anchor_mode else ("TEACH-ALL" if all_mode else "TEACH")
    if all_mode:
        already = sum(1 for w in guide if bot.teach_table.is_taught(w))
        print(f"{label}: {len(guide)} wells, {already} already taught. "
              f"order: {guide[0]} .. {guide[-1]} (snake)\n")
    else:
        print(f"{label} order: {' -> '.join(guide)}\n")
    if anchor_mode:
        print("ANCHOR mode: center the outlet on each corner, press Enter to capture it"
              " (no teaching). After all 4, press q to fit + save the correction.\n"
              "Do NOT press 'h'. Arrows jog; +/- change step.\n")
    elif all_mode:
        print("TEACH-ALL: the arm auto-approaches each well (taught spot if known,"
              " else the model). Nudge to center, Enter to record, n to skip.\n"
              "  - Do NOT press 'h' (it would zero the frame and wreck the wells"
              " you've already taught). The existing calibration stays put.\n"
              "  - 's' saves progress; you can 'q' to quit and rerun --all later to"
              " resume (already-taught wells are re-approached so you can confirm"
              " or 'n' past them).\n")
    else:
        print("Tip: on the FIRST well, center it then press 'h' (home) before Enter."
              " For refine runs (map already built) the arm auto-approaches each well"
              " - just nudge and Enter.\n")

    step_idx = DEFAULT_STEP_IDX
    gi = 0
    recorded = []
    # last manual jog direction per axis (+1/-1; 0 = none since the +X,+Y approach).
    # A taught well must be finished with +X,+Y so goto's +X,+Y creep reproduces it.
    last_dir = {"X": 0, "Y": 0}

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        target = guide[gi] if gi < len(guide) else None
        _announce(target)
        _approach(bot, target, anchor_mode, always=auto_approach)
        last_dir = {"X": 0, "Y": 0}                       # approach ended +X,+Y
        _status(bot, target, STEPS[step_idx], recorded, last_dir)
        while True:
            step = STEPS[step_idx]
            key = _read_key()
            if key == "UP":
                bot.jog_joint(dx=step); last_dir["X"] = 1
            elif key == "DOWN":
                bot.jog_joint(dx=-step); last_dir["X"] = -1
            elif key == "RIGHT":
                bot.jog_joint(dy=step); last_dir["Y"] = 1
            elif key == "LEFT":
                bot.jog_joint(dy=-step); last_dir["Y"] = -1
            elif key == "a":
                bot.jog_joint(dz=step)
            elif key == "z":
                bot.jog_joint(dz=-step)
            elif key in ("+", "="):
                step_idx = min(step_idx + 1, len(STEPS) - 1)
            elif key in ("-", "_"):
                step_idx = max(step_idx - 1, 0)
            elif key == "h":
                if anchor_mode:
                    sys.stdout.write("\n  [h disabled in anchor mode — don't zero the frame]\n")
                else:
                    bot.set_home()
                    sys.stdout.write("\n  [home set here -> 0,0,0]\n")
            elif key == "ENTER":
                if target is None:
                    sys.stdout.write("\n  [no target — press n to pick the next, or q to quit]\n")
                elif anchor_mode:
                    bot.add_anchor(target)
                    if target not in recorded:
                        recorded.append(target)
                    gi += 1
                    target = guide[gi] if gi < len(guide) else None
                    if target is None:
                        sys.stdout.write("  [all 4 corners captured — press q to fit + save]\n")
                    _announce(target)
                    _approach(bot, target, anchor_mode)
                    last_dir = {"X": 0, "Y": 0}
                else:
                    if not _finish_ok(last_dir):
                        sys.stdout.write(
                            "\n  [heads up: last nudge was Down/Left. goto closes in +X,+Y,"
                            " so finishing with Up/Right lands a bit more precisely next"
                            " time. Recorded anyway.]")
                    bot.teach_well(target)
                    recorded.append(target)
                    sys.stdout.write(f"\n  [recorded {target}]  {bot.calibration.summary()}\n")
                    gi += 1
                    target = guide[gi] if gi < len(guide) else None
                    if target is None:
                        sys.stdout.write("  [all wells done — q to save & quit,"
                                         " or keep teaching with n]\n")
                    _announce(target)
                    _approach(bot, target, always=auto_approach)
                    last_dir = {"X": 0, "Y": 0}
            elif key == "n":
                gi += 1
                target = guide[gi] if gi < len(guide) else None
                _announce(target)
                _approach(bot, target, anchor_mode, always=auto_approach)
                last_dir = {"X": 0, "Y": 0}
            elif key == "u":
                if anchor_mode:
                    if recorded:
                        w = recorded.pop()
                        bot._anchor_pts.pop(w, None)
                        gi = max(0, gi - 1)
                        target = guide[gi] if gi < len(guide) else None
                        sys.stdout.write(f"\n  [un-anchored {w}]\n")
                elif recorded:
                    w = recorded.pop()
                    bot.teach_table.forget(w)
                    bot.calibration.reference_points = [
                        p for p in bot.calibration.reference_points if p.well != w]
                    bot.calibration.fit(bot.plate)
                    if bot.well_map:
                        bot.well_map.fit()
                    gi = max(0, gi - 1)
                    target = guide[gi] if gi < len(guide) else None
                    sys.stdout.write(f"\n  [undid {w}]\n")
            elif key == "s":
                if anchor_mode:
                    if bot._anchor_pts:
                        sys.stdout.write("\n")
                        bot.fit_anchor()
                else:
                    p = bot.teach_table.save(bot.teach_path)
                    if bot.calibration.reference_points:
                        bot.calibration.save(bot.calibration_path)
                    sys.stdout.write(f"\n  [saved -> {p}]\n")
            elif key == "q":
                break
            _status(bot, target, STEPS[step_idx], recorded, last_dir)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        if anchor_mode:
            if bot._anchor_pts:
                print("\nfitting frame correction from corners...")
                bot.fit_anchor()
            else:
                print("\nno corners captured — nothing to fit.")
        else:
            bot.teach_table.save(bot.teach_path)
            if bot.calibration.reference_points:
                bot.calibration.save(bot.calibration_path)
            print("\nsaved teach table + metric map. ", bot.teach_table.summary())
        bot.close()


if __name__ == "__main__":
    main()
