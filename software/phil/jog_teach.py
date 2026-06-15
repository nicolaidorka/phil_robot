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
little motion. The console now RECORDS which way your last nudge went per axis and
``goto`` REPLAYS that same finish, so you can center each well however is natural --
finishing Up/Right or Down/Left no longer matters (the machine cancels its own
backlash). The arm auto-approaches each well from the lower-left, so often you only
fine-center and press Enter.

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
from .constants import DEFAULT_BACKEND
from .geometry.well_plate import WellPlate

# Order I guide you through: 4 corners (best spread) + 2 middles to capture the
# arm's curve. You can change this list or just keep going past it.
GUIDE = ["A1", "A12", "H1", "H12", "D6", "E7"]


def _cross(plate):
    """Corner-first, 2-D-spread teach list for a light-but-robust 5-bar fit:
    the 4 corners, then row-A interior, then column-1 interior, then a few interior
    wells. The interior scatter is the key bit -- a BARE row+column cross is nearly
    rank-deficient for the 5-bar's mirror ambiguity (the wrong assembly mode fits the
    cross then blows up in the middle), so we add genuine 2-D spread."""
    rc = {wid: WellPlate.parse_well_id(wid) for wid in plate.well_ids()}
    rows = sorted({r for r, _ in rc.values()})
    cols = sorted({c for _, c in rc.values()})
    byrc = {(r, c): wid for wid, (r, c) in rc.items()}
    if len(rows) < 2 or len(cols) < 2:
        return list(plate.well_ids())
    r0, r1, c0, c1 = rows[0], rows[-1], cols[0], cols[-1]
    order = [byrc.get((r0, c0)), byrc.get((r0, c1)),          # corners (spread first
             byrc.get((r1, c0)), byrc.get((r1, c1))]          # so auto-approach works)
    order += [byrc.get((r0, c)) for c in cols[1:-1]]          # row-A interior
    order += [byrc.get((r, c0)) for r in rows[1:-1]]          # column-1 interior
    for rf, cf in ((0.4, 0.45), (0.3, 0.75), (0.55, 0.3), (0.7, 0.75)):   # interior scatter
        ri = min(len(rows) - 2, max(1, round(rf * (len(rows) - 1))))
        ci = min(len(cols) - 2, max(1, round(cf * (len(cols) - 1))))
        order.append(byrc.get((rows[ri], cols[ci])))
    seen, out = set(), []
    for w in order:
        if w and w not in seen:
            seen.add(w)
            out.append(w)
    return out


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

import select
_keybuf = []                # decoded type-ahead keys read but not yet consumed
_NO_APPROACH = False        # set by --no-approach: never auto-drive the arm


def _read_key():
    """Read one keypress (arrow keys decoded) from a cbreak-mode stdin."""
    if _keybuf:
        return _keybuf.pop(0)
    ch = sys.stdin.read(1)
    if ch == "\x1b":                 # escape sequence (arrow keys)
        seq = sys.stdin.read(2)
        return {"[A": "UP", "[B": "DOWN", "[C": "RIGHT", "[D": "LEFT"}.get(seq, "ESC")
    if ch in ("\r", "\n"):
        return "ENTER"
    return ch


def _coalesce(fd, key, cap=8):
    """Merge already-queued (type-ahead) presses of the SAME key into ONE move, so
    fast taps become a single smooth jog instead of N laggy start/stops -- and nothing
    is dropped (a different key is buffered for the next loop). Capped so a held key
    can't slam one huge move; the remainder just replays on later iterations."""
    n = 1
    while n < cap and select.select([fd], [], [], 0)[0]:
        k = _read_key()
        if k == key:
            n += 1
        else:
            _keybuf.append(k)
            break
    return n


def _status(bot, target, step, recorded, last_dir=None):
    j = bot.joint_position()
    tgt = target if target else "(free)"
    done = ",".join(recorded) if recorded else "none"
    # Show the finish direction goto will replay (0 = no nudge since the +X,+Y
    # approach -> replays +). Purely informational now; any direction is fine.
    fx = (last_dir or {}).get("X", 0); fy = (last_dir or {}).get("Y", 0)
    flag = f"{'+' if fx >= 0 else '-'}X{'+' if fy >= 0 else '-'}Y"
    sys.stdout.write(
        f"\r  TARGET: {tgt:4s} | X={j['X']:+5d} Y={j['Y']:+5d} Z={j['Z']:+5d}"
        f" | step={step:3d} | finish->replay: {flag} | done: {done}        "
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
    if not target or _NO_APPROACH:          # --no-approach: never auto-drive (it can jam)
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
    cross_mode = "--cross" in raw               # corners + row + col + interior scatter
    no_approach = "--no-approach" in raw or "--noapproach" in raw  # skip the auto-drive entirely
    # Backend defaults to the flashed firmware (constants.DEFAULT_BACKEND, = v2);
    # --legacy forces the old driver, --v2 is the (now redundant) explicit opt-in.
    if "--legacy" in raw:
        use_v2 = False
    elif "--v2" in raw:
        use_v2 = True
    else:
        use_v2 = (DEFAULT_BACKEND == "v2")
    raw = [a for a in raw if a not in ("--anchor", "--all", "--cross", "--v2", "--legacy",
                                       "--no-approach", "--noapproach")]
    # --labware <name> and --teach <path>: teach a non-default plate (e.g. a 384)
    # into its own teach file, so 96 and 384 don't co-mingle in one JSON.
    labware = teach = None
    rest, it = [], iter(raw)
    for a in it:
        if a == "--labware":
            labware = next(it, None)
        elif a == "--teach":
            teach = next(it, None)
        else:
            rest.append(a)
    raw = rest
    # When specific wells are named (re-teaching a few), auto-drive to each one
    # like --all does, so you just fine-center instead of jogging there by hand.
    auto_approach = (all_mode or cross_mode or (not anchor_mode and bool(raw))) and not no_approach
    global _NO_APPROACH
    _NO_APPROACH = no_approach

    bot = PhilRobot(backend="v2" if use_v2 else "legacy",
                    labware_path=labware, teach_path=teach)
    # On v2 the joint counts are 32x finer (microsteps, not full-steps), so scale
    # the jog increments to keep the same physical nudge sizes, and STAMP the teach
    # table as v2-scale so goto will accept the freshly-taught data.
    if bot._ustep_scale != 1:
        global STEPS, DEFAULT_STEP_IDX
        # v2 commands in microsteps, so offer FINE jog steps for centering. The legacy
        # list was full-step-limited (~1 mm minimum) -- too coarse for a well. At
        # ~227 microsteps/mm at the tip: 8 ~ 0.035 mm (fine center) ... 2048 ~ 9 mm (travel).
        STEPS = [8, 16, 32, 64, 128, 256, 512, 1024, 2048]
        DEFAULT_STEP_IDX = 6      # 512 ~ 2.3 mm to travel; press '-' for fine centering
        bot.teach_table.ustep_scale = 256
    bot.connect()

    if anchor_mode:
        # Capture the 4 corners as an affine frame correction (no teaching, no refit).
        bot.clear_anchors()
        guide = list(bot.ANCHOR_WELLS)
    elif all_mode:
        # Teach EVERY well on the plate, in a snake order to minimise travel.
        guide = _serpentine(bot.plate)
    elif cross_mode:
        # Light, 2-D-spread set: corners + row A + column 1 + interior scatter.
        guide = _cross(bot.plate)
    else:
        # Optional: pass specific wells to teach/refine, e.g.
        #   python -m phil.jog_teach A6 H6 C1 C12
        guide = [w.upper() for w in raw] or list(GUIDE)

    print(__doc__)
    label = ("ANCHOR" if anchor_mode else "TEACH-ALL" if all_mode
             else "TEACH-CROSS" if cross_mode else "TEACH")
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
        print("Re-teaching one/few wells: jog onto the well center, Enter to record.\n"
              "  - Do NOT press 'h' unless this is a FROM-SCRATCH teach of the FIRST"
              " well -- 'h' ZEROS the whole frame and SHIFTS every other taught well"
              " (that's how a single re-teach can wreck all the others).\n")

    step_idx = DEFAULT_STEP_IDX
    gi = 0
    recorded = []
    # last manual jog direction per axis (+1/-1; 0 = none since the +X,+Y approach).
    # Recorded WITH the well (teach_well finish=...) so goto replays the same backlash
    # engagement -- the operator may finish in any direction.
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
                bot.jog_joint(dx=step * _coalesce(fd, "UP")); last_dir["X"] = 1
            elif key == "DOWN":
                bot.jog_joint(dx=-step * _coalesce(fd, "DOWN")); last_dir["X"] = -1
            elif key == "RIGHT":
                bot.jog_joint(dy=step * _coalesce(fd, "RIGHT")); last_dir["Y"] = 1
            elif key == "LEFT":
                bot.jog_joint(dy=-step * _coalesce(fd, "LEFT")); last_dir["Y"] = -1
            elif key == "a":
                bot.jog_joint(dz=step * _coalesce(fd, "a"))
            elif key == "z":
                bot.jog_joint(dz=-step * _coalesce(fd, "z"))
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
                    # Record which way the last nudge engaged each axis so goto replays
                    # the SAME backlash state (0 = untouched since the +X,+Y approach ->
                    # treat as +). Finish direction no longer needs to be Up/Right.
                    fx = last_dir["X"] if last_dir["X"] != 0 else 1
                    fy = last_dir["Y"] if last_dir["Y"] != 0 else 1
                    bot.teach_well(target, finish=(fx, fy))
                    recorded.append(target)
                    sys.stdout.write(
                        f"\n  [recorded {target}: finish {'+' if fx > 0 else '-'}X,"
                        f"{'+' if fy > 0 else '-'}Y -> goto replays it]  "
                        f"{bot.calibration.summary()}\n")
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
