"""Hardware self-test for the Phil arm robot.

Verifies, in order:
  1. controller discovery over USB,
  2. raw status-packet feedback (the legacy firmware streams 20-byte packets),
  3. connection via the legacy backend (reset / initialize),
  4. joint position feedback,
  5. a small, bounded joint jog with feedback verification, then jog back.

It deliberately does NOT home (homing drives each arm to its limit switch).
Run from the ``software/`` directory:

    python -m phil.selftest                 # connection + feedback only (no motion)
    python -m phil.selftest --move          # also do a small bounded joint jog
    python -m phil.selftest --move --axis Y --usteps 16
    python -m phil.selftest --simulate --move
    python -m phil.selftest --backlash --axis X   # size the goto approach back-off
"""
from __future__ import annotations

import argparse
import time

import serial
import serial.tools.list_ports as lp

from .robot import PhilRobot, PhilHandshakeError
from .constants import CONTROLLER_SN

LEGACY_MSG_LEN = 20


def discover():
    print("== 1. USB controller discovery ==")
    found = None
    for p in lp.comports():
        is_ctrl = (p.manufacturer == "Teensyduino") or (p.description == "Arduino Due")
        if p.manufacturer or (p.description not in (None, "n/a")):
            tag = "  <-- controller" if is_ctrl else ""
            print(f"   {p.device}  desc={p.description!r} mfg={p.manufacturer!r} "
                  f"sn={p.serial_number}{tag}")
        if is_ctrl and found is None:
            found = p
    print("   !! no controller found" if found is None else f"   selected: {found.device}")
    return found


def raw_feedback_probe(device, baud=2000000, msg_len=LEGACY_MSG_LEN, n=40):
    print("\n== 2. Raw feedback probe (controller -> host) ==")
    try:
        s = serial.Serial(device, baud, timeout=1)
    except Exception as e:
        print(f"   could not open {device}: {e}")
        return False
    try:
        time.sleep(0.5)
        s.reset_input_buffer()
        time.sleep(0.3)
        data = s.read(msg_len * n)
        statuses = {data[1 + k * msg_len] for k in range(min(n, len(data) // msg_len))}
        ok = len(data) >= msg_len * 5 and statuses.issubset({0, 1, 2, 3, 4})
        print(f"   received {len(data)} bytes ({msg_len}-byte packets); "
              f"status codes seen: {sorted(statuses)}")
        print(f"   -> feedback streaming: {'PASS' if ok else 'FAIL'}")
        return ok
    finally:
        s.close()


def measure_backlash(bot, axis, sizes=(8, 16, 24, 32, 40), step=8):
    """Semi-automated reversal-gap check (no encoders -> the human is the sensor).

    For each candidate back-off ``t``, run a closed round trip on one axis from the
    current reference: move ``-t``, then creep ``+t`` back to the start count, and
    ask whether the outlet returned to where it started. The smallest ``t`` that
    reproduces the position is the reversal gap; ``goto``'s APPROACH_PRE_USTEPS must
    exceed it so the +X,+Y creep reliably takes up slack on every well. Each test is
    count-neutral (returns to the start count), so the reference is preserved.
    """
    jk = f"d{axis.lower()}"
    print(f"\n== Backlash / approach back-off check on {axis} ==")
    print(f"   Jog the outlet onto a well and center it, then press Enter.")
    input()
    c0 = bot.joint_position()[axis]
    recommended = None
    for t in sizes:
        bot.jog_joint(**{jk: -t})                       # reverse off the reference
        time.sleep(0.2)
        d = t                                           # creep + back to the start count
        while d > 0:
            s = min(step, d); bot.jog_joint(**{jk: s}); time.sleep(0.05); d -= s
        cur = bot.joint_position()[axis]
        ans = input(f"   back-off {t:3d} usteps: did the outlet return to the start "
                    f"(count {cur}, want {c0})? [y/N] ").strip().lower()
        if ans == "y" and recommended is None:
            recommended = t
    if recommended is None:
        print("   none of the tested back-offs reproduced the position cleanly; try "
              "larger sizes or check the mechanics.")
    else:
        margin = recommended + step
        print(f"   -> reversal gap ~{recommended} usteps. Set APPROACH_PRE_USTEPS >= "
              f"{margin} (gap + one step). Current default is "
              f"{bot.APPROACH_PRE_USTEPS}.")
    return recommended


def main(argv=None):
    ap = argparse.ArgumentParser(description="Phil arm hardware self-test")
    ap.add_argument("--move", action="store_true", help="perform a small bounded jog")
    ap.add_argument("--backlash", action="store_true",
                    help="measure the reversal gap to size the goto approach back-off")
    ap.add_argument("--axis", default="X", choices=["X", "Y", "Z"])
    ap.add_argument("--usteps", type=int, default=16, help="jog size in repo usteps (small!)")
    ap.add_argument("--simulate", action="store_true")
    ap.add_argument("--backend", default="legacy", choices=["legacy", "stock", "sim"])
    args = ap.parse_args(argv)

    if not args.simulate:
        ctrl = discover()
        if ctrl is not None:
            raw_feedback_probe(ctrl.device)

    print("\n== 3. Connect (legacy backend: reset / initialize) ==")
    bot = PhilRobot(simulate=args.simulate, backend=args.backend, controller_sn=CONTROLLER_SN)
    try:
        bot.connect()
    except PhilHandshakeError as e:
        print("   HANDSHAKE FAILED:", e)
        return 2
    print("   connected.")

    print("\n== 4. Joint position feedback ==")
    start = bot.joint_position()
    print(f"   joints: X={start['X']} Y={start['Y']} Z={start['Z']} usteps")

    if args.move:
        ax, d = args.axis, args.usteps
        print(f"\n== 5. Bounded joint jog: {ax} +{d} usteps then back ==")
        before = bot.joint_position()[ax]
        bot.jog_joint(**{f"d{ax.lower()}": d})
        time.sleep(0.3)
        moved = bot.joint_position()[ax]
        print(f"   commanded +{d} usteps on {ax}; measured delta = {moved - before:+d} usteps")
        bot.jog_joint(**{f"d{ax.lower()}": -d})
        time.sleep(0.3)
        end = bot.joint_position()[ax]
        print(f"   jogged back; residual from start on {ax} = {end - before:+d} usteps")
        print("   -> joint motion + feedback: PASS" if abs(moved - before) > 0 else
              "   -> no motion detected: CHECK")

    if args.backlash:
        measure_backlash(bot, args.axis)

    bot.close()
    print("\nself-test complete.")


if __name__ == "__main__":
    import sys
    sys.exit(main())
