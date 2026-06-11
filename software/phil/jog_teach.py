"""Real-time arrow-key jog + teach console for the Phil arm.

Run it in YOUR terminal (it needs live keyboard input):

    cd software
    python -m phil.jog_teach

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
little motion. For each well, make your FINAL approach in ONE direction (don't
wiggle right before Enter) and just get the outlet roughly over the well - it
does not need to be perfectly centered.

It tells you which well to go to. Jog the outlet over it, press Enter, repeat.
After a few wells it fits the mm<->joint map (from the labware JSON) so you can
`goto` any well; teach more if any land off.
"""
from __future__ import annotations

import sys
import termios
import time
import tty

from .phil_robot import PhilRobot

# Order I guide you through: 4 corners (best spread) + 2 middles to capture the
# arm's curve. You can change this list or just keep going past it.
GUIDE = ["A1", "A12", "H1", "H12", "D6", "E7"]

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


def _status(bot, target, step, recorded):
    j = bot.joint_position()
    tgt = target if target else "(free)"
    done = ",".join(recorded) if recorded else "none"
    sys.stdout.write(
        f"\r  TARGET: {tgt:4s} | X={j['X']:+5d} Y={j['Y']:+5d} Z={j['Z']:+5d}"
        f" | step={step:3d} | done: {done}        "
    )
    sys.stdout.flush()


def _announce(target):
    if target:
        hint = HINTS.get(target, "")
        sys.stdout.write(f"\n  >>> Go to {target}: {hint}\n")


def _approach(bot, target):
    """If the map is already fitted, drive near the predicted spot so the user
    only has to nudge the last bit before recording."""
    if not target or bot.teach_table.is_taught(target):
        return
    if not (bot.well_map and bot.well_map.is_fitted):
        return
    try:
        p = bot.predict_well(target)
        sys.stdout.write(f"  (auto-approaching predicted {target} -> "
                         f"X={p['X']} Y={p['Y']}; nudge then Enter)\n")
        bot._move_joints_to(x=p["X"], y=p["Y"])
    except Exception as e:
        sys.stdout.write(f"  (auto-approach skipped: {e})\n")


def main(argv=None):
    import sys as _sys
    bot = PhilRobot(backend="legacy")
    bot.connect()

    # Optional: pass specific wells to teach/refine, e.g.
    #   python -m phil.jog_teach A6 H6 C1 C12
    # If the map is already fitted, each is auto-approached so you only nudge.
    guide = [w.upper() for w in (argv if argv is not None else _sys.argv[1:])] or list(GUIDE)

    print(__doc__)
    print(f"Guide order: {' -> '.join(guide)}\n")
    print("Tip: on the FIRST well, center it then press 'h' (home) before Enter."
          " For refine runs (map already built) the arm auto-approaches each well"
          " - just nudge and Enter.\n")

    step_idx = DEFAULT_STEP_IDX
    gi = 0
    recorded = []

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        target = guide[gi] if gi < len(guide) else None
        _announce(target)
        _approach(bot, target)
        _status(bot, target, STEPS[step_idx], recorded)
        while True:
            step = STEPS[step_idx]
            key = _read_key()
            if key == "UP":
                bot.jog_joint(dx=step)
            elif key == "DOWN":
                bot.jog_joint(dx=-step)
            elif key == "RIGHT":
                bot.jog_joint(dy=step)
            elif key == "LEFT":
                bot.jog_joint(dy=-step)
            elif key == "a":
                bot.jog_joint(dz=step)
            elif key == "z":
                bot.jog_joint(dz=-step)
            elif key in ("+", "="):
                step_idx = min(step_idx + 1, len(STEPS) - 1)
            elif key in ("-", "_"):
                step_idx = max(step_idx - 1, 0)
            elif key == "h":
                bot.set_home()
                sys.stdout.write("\n  [home set here -> 0,0,0]\n")
            elif key == "ENTER":
                if target is None:
                    sys.stdout.write("\n  [no target — press n to pick the next, or q to quit]\n")
                else:
                    bot.teach_well(target)
                    recorded.append(target)
                    sys.stdout.write(f"\n  [recorded {target}]  {bot.calibration.summary()}\n")
                    gi += 1
                    target = guide[gi] if gi < len(guide) else None
                    if target is None:
                        sys.stdout.write("  [all wells done — q to save & quit,"
                                         " or keep teaching with n]\n")
                    _announce(target)
                    _approach(bot, target)
            elif key == "n":
                gi += 1
                target = guide[gi] if gi < len(guide) else None
                _announce(target)
                _approach(bot, target)
            elif key == "u":
                if recorded:
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
                p = bot.teach_table.save(bot.teach_path)
                if bot.calibration.reference_points:
                    bot.calibration.save(bot.calibration_path)
                sys.stdout.write(f"\n  [saved -> {p}]\n")
            elif key == "q":
                break
            _status(bot, target, STEPS[step_idx], recorded)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        bot.teach_table.save(bot.teach_path)
        if bot.calibration.reference_points:
            bot.calibration.save(bot.calibration_path)
        print("\nsaved teach table + metric map. ", bot.teach_table.summary())
        bot.close()


if __name__ == "__main__":
    main()
