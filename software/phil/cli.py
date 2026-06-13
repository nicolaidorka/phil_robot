"""Interactive control + teach CLI for the Phil arm robot.

Phil is an articulated 5-bar arm: the X and Y motors are rotary joints whose
two arms meet to hold one outlet over the well plate, and Z raises/lowers it.
Because the joints are rotary (not a Cartesian stage), wells are reached by
*teaching*: jog the outlet over a well once, ``teach`` it, and ``goto`` replays
the saved joint positions. Untaught wells are interpolated from the 4 taught
corners (A1, A12, H1, H12).

Run from the ``software/`` directory::

    python -m phil.cli                 # real hardware (legacy 6-byte firmware)
    python -m phil.cli --simulate      # no hardware; exercise the workflow

Commands:

    joints | where               show the arm's joint positions (usteps)
    jx <usteps> | jy | jz        jog a joint by a relative amount (e.g. jx 400)
    teach <well>                 save current joints as <well> (e.g. teach A1)
    forget <well>                remove a taught well
    goto <well>                  move to a well (coordinated arms, then Z)
    teachpos <name>              save current joints as an off-plate spot (e.g. teachpos WASTE)
    gotopos <name>              move to a named spot (lift -> traverse -> descend, e.g. gotopos WASTE)
    forgetpos <name>            remove a named position
    wellpos <well>               show where a well resolves to (no move)
    travelz [usteps]             set a safe lift height for between-well travel
    scan <w1> <w2> ...           visit several wells in order
    table                        show the teach table

    sethome                      zero the joints at the current pose (manual home)
    metric                       show the fitted 5-bar kinematics / maps + error
    fitkin [starts]              re-fit the 5-bar geometry from taught wells
    check [well]                 go to A1 (or <well>) to VERIFY position by eye
    reanchor [well]              if the check is off (power-cycle/bump): jog onto
                                 A1, run this - recovers the frame (translation), no re-teach
    anchor <well>|fit|clear      sharper edge fix: jog/center each of the 4 corners
                                 (A1 A12 H1 H12), `anchor <corner>` each, then `anchor fit`
    predict <well> [labware]     predicted joints for a well (optionally other labware)
    labware                      list available labware definitions
    save [path]                  persist the teach table to JSON
    home yes                     (CAUTION) run the limit-switch homing sequence
    quit
"""
from __future__ import annotations

import argparse
import sys

from .robot import PhilRobot, PhilHandshakeError


def _jfmt(p: dict) -> str:
    return " ".join(f"{k}={p[k]:+8d}" for k in ("X", "Y", "Z"))


class PhilShell:
    def __init__(self, bot: PhilRobot):
        self.bot = bot

    def do(self, line: str) -> bool:
        parts = line.split()
        if not parts:
            return True
        cmd, args = parts[0].lower(), parts[1:]
        bot = self.bot
        try:
            if cmd in ("quit", "exit", "q"):
                return False
            elif cmd in ("help", "h", "?"):
                print(__doc__)
            elif cmd in ("joints", "where", "pos", "w"):
                print("joints:", _jfmt(bot.joint_position()))
            elif cmd in ("jx", "jy", "jz"):
                bot.jog_joint(**{f"d{cmd[1]}": int(float(args[0]))})
                print("joints:", _jfmt(bot.joint_position()))
            elif cmd == "teach":
                bot.teach_well(args[0])
            elif cmd == "forget":
                bot.teach_table.forget(args[0])
                print(f"forgot {args[0].upper()}")
            elif cmd == "goto":
                bot.goto_well(args[0])
                print("  arrived:", _jfmt(bot.joint_position()))
            elif cmd == "teachpos":
                bot.teach_position(args[0])           # save an off-plate spot (e.g. WASTE)
            elif cmd == "gotopos":
                bot.goto_position(args[0])            # lift -> traverse -> descend to it
                print("  arrived:", _jfmt(bot.joint_position()))
            elif cmd == "forgetpos":
                bot.teach_table.forget_named(args[0])
                print(f"forgot position {args[0].upper()}")
            elif cmd == "wellpos":
                print(_jfmt(bot.well_position(args[0])))
            elif cmd == "travelz":
                bot.set_travel_z(int(float(args[0])) if args else None)
            elif cmd == "scan":
                bot.scan_wells(args, dwell_s=0.3)
            elif cmd == "table":
                print(bot.teach_table.summary())
            elif cmd == "sethome":
                bot.set_home()
            elif cmd in ("metric", "fit"):
                if bot.kin_model and bot.kin_model.is_fitted:
                    print(bot.kin_model.summary())
                if bot.well_map and bot.well_map.is_fitted:
                    print(bot.well_map.summary())
                print(bot.calibration.summary())
            elif cmd == "fitkin":
                bot.fit_kinematics(n_starts=int(args[0]) if args else 400)
            elif cmd == "reanchor":
                bot.reanchor(args[0] if args else None)   # default A1
            elif cmd == "anchor":
                # anchor <well> : capture a corner (jog to center it first)
                # anchor fit / list / clear
                sub = args[0].lower() if args else ""
                if sub == "fit":
                    bot.fit_anchor()
                elif sub == "clear":
                    bot.clear_anchors()
                elif sub in ("list", ""):
                    print("  collected:", ", ".join(sorted(bot._anchor_pts)) or "(none)",
                          "| corners:", ", ".join(bot.ANCHOR_WELLS), "| then: anchor fit")
                else:
                    bot.add_anchor(args[0])
            elif cmd == "check":
                bot.check(args[0] if args else None)      # default A1
            elif cmd == "predict":
                from .geometry.well_plate import WellPlate
                pl = WellPlate.load(" ".join(args[1:])) if len(args) > 1 else None
                print(_jfmt(bot.predict_well(args[0], plate=pl)))
            elif cmd == "labware":
                from .geometry.well_plate import available_labware
                print("current:", bot.plate.display_name)
                print("available:")
                for n in available_labware():
                    print("  -", n)
            elif cmd == "save":
                path = bot.teach_table.save(args[0] if args else bot.teach_path)
                print(f"saved teach table -> {path}")
                if bot.calibration.is_fitted and bot.calibration.reference_points:
                    cpath = bot.calibration.save(bot.calibration_path)
                    print(f"saved metric map  -> {cpath}")
            elif cmd == "home":
                if not args or args[0] != "yes":
                    print("CAUTION: homing drives each arm to its limit switch. Make sure "
                          "the workspace is clear, then run:  home yes")
                else:
                    bot.home()
            else:
                print(f"unknown command: {cmd!r} (type 'help')")
        except (IndexError, ValueError) as e:
            print(f"error: {e}")
        except Exception as e:                       # keep the shell alive
            print(f"error: {type(e).__name__}: {e}")
        return True

    def repl(self):
        print("\nPhil arm shell. Type 'help' for commands, 'quit' to exit.")
        print(self.bot.teach_table.summary())
        print("Tip: jog with jx/jy/jz, 'teach <well>' to save, 'goto <well>' to replay.\n")
        while True:
            try:
                line = input("phil> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not self.do(line):
                break


def main(argv=None):
    ap = argparse.ArgumentParser(description="Phil arm control + teach CLI")
    ap.add_argument("--simulate", action="store_true", help="no hardware; in-memory backend")
    ap.add_argument("--labware", default=None, help="path to labware JSON")
    ap.add_argument("--teach", default=None, help="path to teach-table JSON")
    ap.add_argument("--backend", default="legacy",
                    choices=["legacy", "v2", "stock", "sim"])
    ap.add_argument("--version", default="Teensy", help="controller version")
    ap.add_argument("-c", "--command", action="append", default=[],
                    help="run a command then continue (repeatable)")
    ap.add_argument("--no-check", action="store_true",
                    help="skip the startup/shutdown A1 verification")
    args = ap.parse_args(argv)

    bot = PhilRobot(labware_path=args.labware, teach_path=args.teach,
                    simulate=args.simulate, backend=args.backend,
                    controller_version=args.version)
    try:
        bot.connect()
    except PhilHandshakeError as e:
        print(f"\n{e}\n")
        return 2
    shell = PhilShell(bot)
    do_check = not (args.no_check or args.simulate)
    try:
        # HABIT: verify on the anchor well (A1) at startup
        if do_check:
            if bot.frame_suspect:
                print(f"\n[startup] frame looks reset — jog the outlet onto "
                      f"{bot.ANCHOR_WELL} and run `reanchor {bot.ANCHOR_WELL}` "
                      "before any goto.\n")
            else:
                print(f"\n[startup check] moving to {bot.ANCHOR_WELL} to verify...")
                bot.check()
        for c in args.command:
            print(f"phil> {c}")
            shell.do(c)
        shell.repl()
    finally:
        # HABIT: park on the anchor well (A1) when done, so next start is verified
        try:
            if do_check and bot.connected and not bot.frame_suspect:
                print(f"[shutdown] parking on {bot.ANCHOR_WELL} (anchor) "
                      "and saving position...")
                bot.goto_well(bot.ANCHOR_WELL)
        except Exception:
            pass
        bot.close()


if __name__ == "__main__":
    sys.exit(main())
