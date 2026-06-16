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
    sweep <w1> <w2> ...          goto each + report predicted-vs-reached error (validate the fit)
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

Sibling tools (run from the shell as `python3 -m phil.<tool>`):
    tiptrack                     live window of the converged tip over the plate;
                                 jog + Enter to lock a well + f to fitkin (--simulate for a demo)
    stepcheck                    flag mis-taught wells whose joints break the lattice
"""
from __future__ import annotations

import argparse
import sys

from .robot import PhilRobot, PhilHandshakeError
from .constants import DEFAULT_BACKEND, CONTROLLER_SN


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
            elif cmd == "sweep":
                if not args:
                    print("usage: sweep <well> [well ...]   (e.g. sweep D6 E7 F7 H6 H12)")
                else:
                    self._sweep([w.upper() for w in args])
            elif cmd == "tour":
                # visit EVERY well in snake order, pausing at each so you can eyeball it
                dwell = float(args[0]) if args else 5.0
                self._tour(dwell)
            elif cmd == "table":
                print(bot.teach_table.summary())
            elif cmd == "sethome":
                bot.set_home()
            elif cmd == "rehome":
                # recovery: jog onto A1 first, then this restores the whole frame
                bot.rehome()
            elif cmd in ("metric", "fit"):
                if bot.kin_model and bot.kin_model.is_fitted:
                    print(bot.kin_model.summary())
                if bot.well_map and bot.well_map.is_fitted:
                    print(bot.well_map.summary())
                print(bot.calibration.summary())
            elif cmd == "fitkin":
                bot.fit_kinematics(n_starts=int(args[0]) if args else 400)
            elif cmd == "gridcheck":
                bot.grid_check(tol_mm=float(args[0]) if args else 1.5)
            elif cmd == "gridloo":
                rows = bot.teach_table.grid_loo(bot.plate)
                if not rows:
                    print("  gridloo: need >=4 taught wells to check.")
                else:
                    n = int(args[0]) if args else 8
                    print("  grid leave-one-out vs the rigid JSON grid (worst first; "
                          "large usteps = likely mis-taught / wrong-frame well):")
                    for w, err in rows[:n]:
                        print(f"    {w:4s}  {err:8.1f} usteps")
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

    def _tour(self, dwell):
        """Visit every well in snake order, pausing `dwell` s at each so you can confirm
        by eye. Prints which well + how it resolved (taught vs kinematics). Ctrl-C stops."""
        import time
        from .jog_teach import _serpentine
        bot = self.bot
        order = _serpentine(bot.plate)
        print(f"touring {len(order)} wells, {dwell}s each (snake order). Ctrl-C to stop.\n")
        try:
            for i, w in enumerate(order, 1):
                _, src = bot._resolve_well(w)
                bot.goto_well(w)
                print(f"  [{i:2d}/{len(order)}] AT {w:4s} [{src}] — confirm ({dwell:.0f}s)")
                time.sleep(dwell)
            bot.goto_well("A1")
            print("  tour done -> parked on A1.")
        except KeyboardInterrupt:
            print("\n  tour stopped.")

    def _sweep(self, wells):
        """Drive to each well and report the open-loop error: predicted joints (from
        the model, or exact for a taught well) vs the joints actually reached, in
        usteps AND true tip-mm (via the 5-bar forward map, since the joints are
        rotary -- a ustep is not a fixed mm). This is the camera-free validation:
        interior wells test the model; taught wells (~0) confirm goto replay. A
        large or sign-flipping residual across wells is the mirror/sag tell."""
        bot = self.bot
        kin = bot.kin_model if (bot.kin_model and bot.kin_model.is_fitted) else None
        if kin is None:
            print("  (no fitted kinematics -- run fitkin first; tip-mm unavailable)")
        print(f"  {'well':4s} {'predX':>7s} {'predY':>7s} {'reachX':>7s} {'reachY':>7s}"
              f" {'dX':>6s} {'dY':>6s} {'tip_mm':>7s}")
        rows = []
        for w in wells:
            try:
                pred = bot.well_position(w)
            except Exception as e:
                print(f"  {w:4s}  (predict failed: {e})")
                continue
            bot.goto_well(w)
            rx, ry = bot._read_joints_settled()
            dx, dy = rx - pred["X"], ry - pred["Y"]
            tip = ""
            if kin is not None:
                ep, er = kin.forward(pred["X"], pred["Y"]), kin.forward(rx, ry)
                if ep is not None and er is not None:
                    mm = float(((er[0] - ep[0]) ** 2 + (er[1] - ep[1]) ** 2) ** 0.5)
                    tip = f"{mm:.2f}"
                    rows.append((w, dx, dy, mm))
            tag = " [taught]" if bot.teach_table.is_taught(w) else ""
            print(f"  {w:4s} {pred['X']:7d} {pred['Y']:7d} {rx:7d} {ry:7d}"
                  f" {dx:6d} {dy:6d} {tip:>7s}{tag}")
        if rows:
            mms = [r[3] for r in rows]
            worst = max(rows, key=lambda r: r[3])
            print(f"\n  tip error mm: mean {sum(mms) / len(mms):.2f}, "
                  f"worst {worst[3]:.2f} @ {worst[0]}  "
                  f"(target <~1 mm; >2 mm or dX/dY flipping sign across wells = check the model)")

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
    ap.add_argument("--backend", default=DEFAULT_BACKEND,
                    choices=["legacy", "v2", "stock", "sim"])
    ap.add_argument("--version", default="Teensy", help="controller version")
    ap.add_argument("-c", "--command", action="append", default=[],
                    help="run a command then continue (repeatable)")
    ap.add_argument("--no-check", action="store_true",
                    help="skip the startup/shutdown A1 verification")
    args = ap.parse_args(argv)

    bot = PhilRobot(labware_path=args.labware, teach_path=args.teach,
                    simulate=args.simulate, backend=args.backend,
                    controller_version=args.version,
                    controller_sn=CONTROLLER_SN)   # bind to Phil's board, never the microscope's
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
        # HABIT: park on the anchor well (A1) when done, so next start is verified.
        # WATCHDOG: an EMI USB drop can make this move grind through long per-command
        # timeouts (move_timeout_s) -- minutes that read as "the CLI won't quit" (hit live
        # 2026-06-15). Run the park in a daemon thread and bound it; if it overruns, skip
        # and close anyway. The MCU keeps the frame, so a skipped park costs nothing but
        # the next-start A1 re-check.
        try:
            if do_check and bot.connected and not bot.frame_suspect:
                print(f"[shutdown] parking on {bot.ANCHOR_WELL} (anchor) "
                      "and saving position...")
                import threading
                done = threading.Event()

                def _park():
                    try:
                        bot.goto_well(bot.ANCHOR_WELL)
                    except Exception:
                        pass
                    finally:
                        done.set()

                threading.Thread(target=_park, daemon=True).start()
                if not done.wait(timeout=15):
                    print("[shutdown] park timed out (USB link dropped?) — skipping; "
                          "frame is preserved, just re-check A1 next start.")
        except Exception:
            pass
        bot.close()


if __name__ == "__main__":
    sys.exit(main())
