# Operating Phil — start here

Phil is an **open-loop 5-bar arm** that positions one nozzle over the **X/Y centre of
any well of a 96-well plate**. X and Y are **rotary arm joints** (a parallel linkage),
Z is up/down. It is **not** a microscope — it only reuses the Squid Teensy motor
firmware. Full background + code map: [`.claude/CLAUDE.md`](.claude/CLAUDE.md) and
[`.claude/ARCHITECTURE.md`](.claude/ARCHITECTURE.md).

> **For the next Claude / operator:** before changing code or calibration, read
> [`.claude/RULES.md`](.claude/RULES.md) and [`.claude/LEARNINGS.md`](.claude/LEARNINGS.md)
> (hard rules + every past mistake). This file is the practical "how to use it now."

---

## Current state (2026-06-15)

- ✅ **All 96 wells taught** in a stable v2 frame
  (`software/phil/config/phil_teach.json`, `ustep_scale: 256`). Verified clean —
  median ~0.7 mm local residual, worst ~2.4 mm, i.e. **at the hardware floor**.
- ✅ Committed + pushed to **`master`** (the repo's primary branch — there is no `main`).
  The teach data travels with the repo.
- ✅ **WASTE position taught** (`named: WASTE`, off-plate at the container rim, **hardware-verified**
  well→WASTE→well). It's a **dispense-from-ABOVE** spot — the nozzle holds at the rim height and
  never descends *into* the waste (the 5-bar arms would hit the tall wall). Reach it with
  `gotopos WASTE` (CLI) or `g WASTE` (drive.py).
- ✅ **Tall-obstacle clearance handled in code:** `goto`/`gotopos` lift to
  **`max(travelz, current_Z, target_Z)`** (`_safe_move`), so moves *to/from* the tall waste ride
  at rim height — clearing the wall both entering AND leaving — while plate well-to-well moves
  stay low. `travelz` stays 0 (fine — the waste uses its own height); set a small `travelz` only
  if you want extra well-clearance margin.
- ✅ A bump → **`reanchor A1` recovers the wells AND the WASTE together** (WASTE is stored as a
  model coordinate, like the wells, so the reanchor translation applies to it too — never re-teach).
- ⚠️ The Z axis has **no soft limit** — jog Z up only modestly (small steps); over-jogging stalls
  the motor and hangs the console (see LEARNINGS 2026-06-15).

## Connect & use

Run from `software/`:

```bash
cd software
python3 -m phil.cli          # backend defaults to v2 (the flashed firmware)
phil> check                  # moves to A1 — eyeball that the nozzle is centred on A1
phil> goto B10               # a taught well replays its EXACT recorded joints
```

The Teensy is **auto-detected by serial SN 16640550** — never pass a port (the
`/dev/ttyACMx` number is not stable). **Only run one program on the port at a time**
(the v2 driver takes an exclusive lock; a second console is refused).

## Moving to a fresh laptop / new machine

The teach **data** is in git; the joint **frame** is machine-specific and is *not*
(by design — a stale frame would be wrong on a new setup). So:

1. `git clone` the repo → `pip install -r requirements.txt`
2. Connect Phil, run `check`.
3. If the nozzle is **not** on A1 (the Teensy was power-cycled or the rig moved):
   `python3 -m phil.drive` → jog the nozzle onto A1's centre → press **`r`** (reanchor).
   ~30 seconds.
4. Done — all 96 wells work via `goto`. **You never re-teach.**

## Recovery after a bump / power-cycle

A nudge or power-cycle shifts the whole frame ~uniformly — it does **not** destroy the
teaching. One-step fix:

- `python3 -m phil.drive` → jog the nozzle onto A1's centre → press **`r`** (reanchor).
- **Why it works:** every well is stored relative to **one shared frame**, so re-pinning
  A1 re-pins all 96 (their relative geometry is preserved). `reanchor` writes only a
  *translation* correction; it never touches the teach data. (It's a pure translation —
  for a sharper far-edge fix after a big/twisty disturbance, anchor the 4 corners:
  `anchor A1` / `A12` / `H1` / `H12`, then `anchor fit`.)
- The startup/shutdown **A1 `check`** is how you catch a shift (no encoder → a
  mid-session bump isn't auto-detected; confirm by eye).

## Hard rules — do not violate

- ⛔ **Never press `h`** in a teach/drive session — it zeros the frame and wrecks every
  taught well. (Only correct on the very first well of a from-scratch *geometry* re-teach.)
- ⛔ **Never run `fitkin` after teaching.** It refits the **retired, non-convex 5-bar**
  and can **regress a good calibration**; taught wells short-circuit the model anyway,
  and untaught wells use the rigid grid. After teaching, the only steps that matter are:
  **set `travelz`, teach `WASTE`**. (`fitkin` is *only* for a genuine physical
  geometry change.)
- ⛔ **Never hand-move the arms** to "set" a position — open-loop, not tracked, loses
  steps. Position changes only via commanded jogs.
- **Jog small** (rotary joints are very sensitive). **Don't blind-fire HOME** /
  limit-switch homing (unverified on this firmware).

## The hardware floor (set expectations)

Repeatability is ~**1–2 mm** — one full-step detent at the tip, because the 560 mA
microstep holding torque can't hold intermediate positions under the heavy 5-bar arm.
You **cannot** nudge below this by eye or software; it's a hardware change (more motor
current / less backlash / an encoder — **not** a camera, which is banned). All
accuracy talk is **X/Y only**; ignore Z unless the operator explicitly raises it.

## How `goto` resolves a well

`taught` (exact replay) → **rigid-grid predictor** (untaught) → curve-fit → affine →
5-bar kinematics (**retired, dead-last**). With all 96 taught, only the first applies.

## Teaching more / re-checking wells

- Teach or redo: `python3 -m phil.jog_teach --all` (all 96, snake order, resumable:
  `s` save, `q` quit) or `python3 -m phil.jog_teach B1 C1` (just those wells). Arrow
  keys jog one motor; `+`/`-` change step; **Enter** records; `n` skips; **never `h`**.
  Centre in any direction — the finish backlash is recorded and `goto` replays it.
- **Saves are protected:** `TeachTable.save()` snapshots the file to a timestamped
  `.backup-*` before every overwrite (newest 30 kept) and **refuses a catastrophic
  shrink** (`TeachShrinkGuard` — preserves the on-disk file, writes a `.rejected-save`
  sidecar; override with `save(allow_shrink=True)`). Recover from the newest backup.
- **Find mis-taught wells WITHOUT `fitkin`:** the rigid-grid leave-one-out
  `TeachTable.grid_loo(plate)`, or a local second-difference check (a well should sit
  ~midway between opposite neighbours). Do **not** use `phil/stepcheck.py` — it
  requires `fitkin`, which we don't run.

## The dispense cycle (DONE — WASTE taught, lift handled)

The full cycle works on hardware: `goto <well>` → *(liquid handling, outside this code)* →
`gotopos WASTE` → repeat. Moves to/from the waste auto-lift to the rim and hold there; well
moves stay low. Nothing more to set up.

**Re-teaching the WASTE (e.g. if the container moves)** — use `drive.py` (arrow keys + Z):

```bash
python3 -m phil.drive
#  jog Z UP modestly in SMALL steps (press '-' first) to the waste-rim height — do NOT
#    over-jog (no soft limit -> stall/hang). Jog X/Y over the waste opening.
#  press 't', type WASTE   -> saves the rim pose (stored as a model coord; survives reanchor)
#  press 'g', type WASTE   -> verify (lifts to rim, traverses over, holds)
```
The waste is **dispense-from-above** — it holds at the rim, never descends in. `travelz` is a
separate global *floor* for the lift; leave it 0 or set a small well-clearance value with
`v` (drive.py) / `travelz <usteps>` (CLI).

## Deeper docs (in `.claude/`)

`CLAUDE.md` (overview + code map) · `RULES.md` (operating rules) · `TROUBLESHOOTING.md` ·
`LEARNINGS.md` (past mistakes — read before changing things) · `ARCHITECTURE.md` ·
`UNITS-AND-CALIBRATION.md` · `V2-FIRMWARE-NOTES.md` · `FINDINGS.md`.
