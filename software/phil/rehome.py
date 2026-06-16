"""Foolproof re-home: the ONE blessed recovery. It does exactly one thing -- jog,
then zero the frame at A1 -- so it can't corrupt the teach table the way the teach
console can (no well recording, no 'advance', no way to shift the frame after the
fact). A1 is taught at (0,0), so zeroing here restores the ENTIRE taught frame; every
well is relative to this zero. NO re-teach, ever.

    cd software
    python3 -m phil.rehome --v2

Steps it walks you through:
  1. jog the nozzle to the TRUE CENTRE of well A1  (arrows = X/Y, a/z = Z, +/- = step)
  2. press ENTER -> sets home here -> saves. Done.
  3. it then drives to A1 to confirm (a no-op if you're centred).

ONLY ever run this with the nozzle going onto A1. It zeroes wherever it is.
"""
from __future__ import annotations

import os
import sys
import termios
import tty

from .robot import PhilRobot
from .constants import DEFAULT_BACKEND, CONTROLLER_SN


def _read_key():
    # Read from the RAW fd (os.read), NOT sys.stdin.read -- the latter keeps a Python
    # read-ahead buffer that tcflush can't clear, so keys pressed during a slow move
    # would survive the flush and fire in a burst.
    fd = sys.stdin.fileno()
    ch = os.read(fd, 1)
    if ch == b"\x1b":
        seq = os.read(fd, 2)
        return {b"[A": "UP", b"[B": "DOWN", b"[C": "RIGHT", b"[D": "LEFT"}.get(seq, "ESC")
    if ch in (b"\r", b"\n"):
        return "ENTER"
    try:
        return ch.decode()
    except UnicodeDecodeError:
        return "ESC"


def main(argv=None):
    raw = list(argv if argv is not None else sys.argv[1:])
    if "--legacy" in raw:
        use_v2 = False
    elif "--v2" in raw:
        use_v2 = True
    else:
        use_v2 = (DEFAULT_BACKEND == "v2")
    raw = [a for a in raw if a not in ("--v2", "--legacy")]
    # --anchor WELL: reanchor on a GOOD well (applies a frame translation to ALL wells)
    # instead of zeroing A1. Use this when A1/home is the corrupt point. Pick a well the
    # gridcheck says is on-grid (e.g. H12, D6).
    anchor_well = None
    if "--anchor" in raw:
        i = raw.index("--anchor")
        anchor_well = raw[i + 1].upper() if i + 1 < len(raw) else "H12"
    bot = PhilRobot(backend="v2" if use_v2 else "legacy", controller_sn=CONTROLLER_SN)
    steps = [8, 16, 32, 64, 128, 256, 512, 1024] if use_v2 else [8, 16, 32, 64, 120]
    si = 3
    if use_v2:
        bot.teach_table.ustep_scale = 256
    bot.connect()
    print(__doc__)
    if anchor_well:
        print(f">>> REANCHOR mode on {anchor_well}. Driving near it; jog to its TRUE centre, "
              f"then ENTER to reanchor (shifts the whole frame to match). q = cancel\n")
        bot.teach_table.z_travel_usteps = bot.teach_table.z_travel_usteps or 30000
        try:
            bot.goto_well(anchor_well)
        except Exception as e:
            print(f"  (couldn't pre-drive to {anchor_well}: {e})")
    else:
        print(">>> Jog the nozzle to the CENTRE of A1, then press ENTER to set home. (q = cancel)\n")

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    homed = False
    try:
        tty.setcbreak(fd)
        while True:
            j = bot.joint_position()
            sys.stdout.write(f"\r  step={steps[si]:4d}  joints X={j['X']:+6d} "
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
                if anchor_well:
                    cx, cy = bot.reanchor(anchor_well)   # frame translation to match this well
                    print(f"\n  reanchored on {anchor_well}: frame shift cx={cx:+.0f} cy={cy:+.0f}.")
                else:
                    bot.set_home()              # zero at A1 -> whole taught frame restored
                homed = True
                break
            elif k == "q":
                break
            if moved:
                # discard any keys pressed WHILE the (slow) move ran, so presses don't
                # pile up in the tty buffer and then fire in a burst. One press = one move.
                termios.tcflush(fd, termios.TCIFLUSH)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    if homed and anchor_well:
        print(f"\n  REANCHORED on {anchor_well} -> the frame is shifted to match. Every well "
              f"(taught + model) should now land. Verify with goto.")
    elif homed:
        print("\n  HOME SET at A1 -> all 24 taught wells restored (no re-teach).")
        print("  confirming with goto A1 ...")
        try:
            bot.goto_well("A1")
            print("  -> the nozzle should be on A1. If not, rerun and centre more carefully.")
        except Exception as e:
            print(f"  (goto A1 check skipped: {e})")
    else:
        print("\n  cancelled — frame unchanged.")
    bot.close()


if __name__ == "__main__":
    main()
