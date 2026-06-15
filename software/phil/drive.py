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
    g               go to a well (e.g. D6) OR a named position (e.g. WASTE) — lift/traverse/descend
    r               reanchor: lock the whole frame to A1 (jog onto A1's center first,
                    then press r — recovers a shifted frame, no re-teach)
    t               log the CURRENT pose as a named off-plate position (e.g. WASTE)
    v               set the travel-Z lift to the CURRENT Z (gotopos lifts here FIRST, then over)
    p               print the current joint position
    q               quit

This is a SAFE driver: it never teaches WELLS, never homes/zeros the frame, and never
writes calibration — so the taught wells are never disturbed. `t` saves a NAMED
off-plate position and `v` sets the travel-Z lift; both are operational settings (not
well teaching) and are persisted to the teach file. Jog small (rotary + backlash).

  Teach a WASTE spot:  press `-` to a SMALL step first. Jog Z up only a MODEST amount
  (a few taps — just enough to clear the tallest wall). Do NOT hold `a` / over-jog: the Z
  axis has NO soft limit and will stall + hang the console. Press `v` (sets the lift) →
  jog over the waste opening and down to dispense height, press `t` and type WASTE →
  verify with `g WASTE` (lift → traverse → descend, "up first, then over").
"""
from __future__ import annotations

import select
import sys
import termios
import time
import tty

from .robot import PhilRobot
from .constants import DEFAULT_BACKEND

# Jog sizes in repo usteps. The firmware moves in whole full-steps (8 usteps each,
# ~1 mm of outlet travel near the plate), so these are multiples of 8.
STEPS = [8, 16, 32, 64, 120]
DEFAULT_STEP_IDX = 1


_keybuf = []                # decoded type-ahead keys read but not yet consumed


def _read_key():
    if _keybuf:
        return _keybuf.pop(0)
    ch = sys.stdin.read(1)
    if ch == "\x1b":                      # escape sequence (arrow keys)
        seq = sys.stdin.read(2)
        return {"[A": "UP", "[B": "DOWN", "[C": "RIGHT", "[D": "LEFT"}.get(seq, "ESC")
    if ch in ("\r", "\n"):
        return "ENTER"
    return ch


def _coalesce(fd, key, cap=8):
    """Merge already-queued (type-ahead) presses of the SAME key into ONE move, so
    fast taps become a single smooth jog instead of N laggy start/stops -- and nothing
    is dropped (a different key is buffered for the next loop)."""
    n = 1
    while n < cap and select.select([fd], [], [], 0)[0]:
        k = _read_key()
        if k == key:
            n += 1
        else:
            _keybuf.append(k)
            break
    return n


def _status(bot, step):
    j = bot.joint_position()
    sys.stdout.write(
        f"\r  X={j['X']:+6d} Y={j['Y']:+6d} Z={j['Z']:+6d} | step={step:3d} usteps "
        f"| move  a/z=Z  +/-=step  g=goto  r=reanchor  t=log-pos  v=set-travelZ  q=quit  "
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

    bot = PhilRobot(backend="sim" if simulate else DEFAULT_BACKEND, simulate=simulate)
    bot.connect()
    # On v2 the joints are 32x finer (microsteps), so the legacy full-step ladder
    # tops out at ~0.5 mm -- painfully slow to cross a well during a frame recovery.
    # Scale the jog ladder the same way jog_teach does: fine-center .. fast travel.
    if getattr(bot, "_ustep_scale", 1) != 1:
        global STEPS, DEFAULT_STEP_IDX
        STEPS = [8, 16, 32, 64, 128, 256, 512, 1024, 2048]   # ~0.035 .. ~9 mm at the tip
        DEFAULT_STEP_IDX = 6                                  # 512 ~ 2.3 mm; '-' for fine
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
                bot.jog_joint(dx=step * _coalesce(fd, "UP"))
            elif key == "DOWN":
                bot.jog_joint(dx=-step * _coalesce(fd, "DOWN"))
            elif key == "RIGHT":
                bot.jog_joint(dy=step * _coalesce(fd, "RIGHT"))
            elif key == "LEFT":
                bot.jog_joint(dy=-step * _coalesce(fd, "LEFT"))
            elif key == "a":
                bot.jog_joint(dz=step * _coalesce(fd, "a"))
            elif key == "z":
                bot.jog_joint(dz=-step * _coalesce(fd, "z"))
            elif key in ("+", "="):
                step_idx = min(step_idx + 1, len(STEPS) - 1)
            elif key in ("-", "_"):
                step_idx = max(step_idx - 1, 0)
            elif key == "r":
                # Re-lock the frame to A1. Jog the nozzle onto A1's TRUE center first,
                # then press r. Pure-translation correction -> reliable (not SET_POSITION),
                # recovers a shifted frame with no re-teaching.
                sys.stdout.write("\n  reanchoring frame to A1 at the current pose...\n")
                try:
                    cx, cy = bot.reanchor("A1")
                    sys.stdout.write(f"  [frame locked to A1: translation cx={cx:+.0f} "
                                     f"cy={cy:+.0f} usteps]\n")
                except Exception as e:
                    sys.stdout.write(f"  [reanchor failed: {e}]\n")
            elif key == "g":
                name = _prompt_line(fd, old, "  go to well / named pos (e.g. D6, WASTE): ").strip().upper()
                if name:
                    try:
                        if bot.teach_table.is_named(name):
                            bot.goto_position(name)           # lift -> traverse -> descend
                        else:
                            bot.goto_well(name)
                    except Exception as e:
                        sys.stdout.write(f"  [goto failed: {e}]\n")
            elif key == "t":
                # Log the CURRENT pose as a named off-plate position (e.g. WASTE). Does NOT
                # touch taught wells/calibration -- adds to teach_table.named, then PERSISTS
                # (teach_position itself does not save to disk).
                name = _prompt_line(fd, old, "  save current pose as named position (e.g. WASTE): ").strip().upper()
                if name:
                    bot.teach_position(name)
                    bot.teach_table.save(bot.teach_path)
                    sys.stdout.write(f"  [saved '{name}' & persisted -> reach with 'g {name}' / CLI 'gotopos {name}']\n")
            elif key == "v":
                # Capture the CURRENT Z as the travel-Z lift. Jog Z ALL THE WAY UP first,
                # THEN press v -> goto/gotopos lift to here before traversing ("up then over").
                bot.set_travel_z()
                bot.teach_table.save(bot.teach_path)
                sys.stdout.write(f"\n  [travel-Z lift = {bot.teach_table.z_travel_usteps} usteps & persisted "
                                 f"-> gotopos lifts ALL the way up here first, then over]\n")
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
