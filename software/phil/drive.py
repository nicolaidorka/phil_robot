"""Free arrow-key drive for the Phil arm — just move it around, no teaching.

Run it in YOUR terminal (it needs live keyboard input):

    cd software
    python -m phil.drive                 # real hardware
    python -m phil.drive --simulate      # no hardware

Keys:
    Up / Down       X arm  +/-   (one whole step per press)
    Left / Right    Y arm  +/-
    a / z           Z up / down
    + / -           bigger / smaller jog step
    g               go to a well (type the well id, e.g. D6, then Enter)
    p               print the current joint position
    q               quit

This is a SAFE driver: it never teaches, never homes/zeros the frame, and never
writes calibration — so you can move the arm freely without disturbing the taught
wells. The joints are rotary with backlash, so jog small.
"""
from __future__ import annotations

import sys
import termios
import time
import tty

from .robot import PhilRobot

# Jog sizes in repo usteps. The firmware moves in whole full-steps (8 usteps each,
# ~1 mm of outlet travel near the plate), so these are multiples of 8.
STEPS = [8, 16, 32, 64, 120]
DEFAULT_STEP_IDX = 1


def _read_key():
    ch = sys.stdin.read(1)
    if ch == "\x1b":                      # escape sequence (arrow keys)
        seq = sys.stdin.read(2)
        return {"[A": "UP", "[B": "DOWN", "[C": "RIGHT", "[D": "LEFT"}.get(seq, "ESC")
    if ch in ("\r", "\n"):
        return "ENTER"
    return ch


def _status(bot, step):
    j = bot.joint_position()
    sys.stdout.write(
        f"\r  X={j['X']:+6d} Y={j['Y']:+6d} Z={j['Z']:+6d} | step={step:3d} usteps "
        f"| arrows=move  a/z=Z  +/-=step  g=goto  p=pos  q=quit     "
    )
    sys.stdout.flush()


def _prompt_line(fd, old, msg):
    """Temporarily leave cbreak to read a full typed line (e.g. a well id)."""
    termios.tcsetattr(fd, termios.TCSADRAIN, old)
    try:
        sys.stdout.write("\n" + msg)
        sys.stdout.flush()
        return sys.stdin.readline().strip()
    finally:
        tty.setcbreak(fd)


def main(argv=None):
    import sys as _sys
    raw = list(argv if argv is not None else _sys.argv[1:])
    simulate = "--simulate" in raw

    bot = PhilRobot(backend="sim" if simulate else "legacy", simulate=simulate)
    bot.connect()
    print(__doc__)

    step_idx = DEFAULT_STEP_IDX
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        _status(bot, STEPS[step_idx])
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
            elif key == "g":
                well = _prompt_line(fd, old, "  go to well: ")
                if well:
                    try:
                        bot.goto_well(well)
                    except Exception as e:
                        sys.stdout.write(f"  [goto failed: {e}]\n")
            elif key == "p":
                j = bot.joint_position()
                sys.stdout.write(f"\n  joints: X={j['X']} Y={j['Y']} Z={j['Z']} usteps\n")
            elif key == "q":
                break
            _status(bot, STEPS[step_idx])
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("\ndone (nothing saved — free drive).")
        bot.close()


if __name__ == "__main__":
    main()
