# Phil v2 goto — root-cause fixes (2026-06-13) + hardware test plan

After the microstep reflash, `goto` landed taught wells "way off" and `goto A1` drove
OFF the plate. This was NOT a kinematics problem — taught wells replay their exact
recorded joints, so a miss means the MOTION/FRAME is wrong. The 5-bar model itself is
good (RMS 0.31 mm, interior leave-one-out 0.43 mm; physical geometry l~65.4/63.0,
dist~140.7/149.0). Root cause: **the v2 firmware HONORS velocity/accel (legacy ignored
it)**, which turned several legacy-era assumptions into bugs.

## What was wrong, and the fix (all in `software/phil/`)

| # | Bug | Fix |
|---|-----|-----|
| 1 | **Step loss** — vel/accel "mm" are notional (×~12800 usteps/"mm"); connect set vel=12/accel=150 ≈ 750 mm/s tip → heavy 5-bar skips steps; counter still reads on-target (no encoder) → error compounds across a sweep. | `robot.py` connect: default **vel=2.0 accel=15** (env `PHIL_VMAX`/`PHIL_AMAX`). accel matters most. |
| 2 | **Chunk jerk** — `_move_joints_to` split traverses into ~11 stop/start legs (for legacy fixed-profile) → each restart a skip chance on v2. | v2 issues **one smooth coordinated MOVETO**; legacy keeps chunking. |
| 3 | **Off-plate run-up** — `_approach_joints` pre-positions to joint−PRE; at corner A1 (0,0) that's negative joints → off plate. Firmware MOVETO is NOT clamped. | `_joint_bounds()`/`_clamp_joint()` clamp every commanded joint to the taught box (margin 0). PRE reduced 2560→500. Verified: all 96 wells in-bounds. |
| 4 | **Blind closed loop** — v2 read-back follows commanded count, can't see step-loss, so the 6-iter correction re-drove the run-up. | v2 does **one clean directional approach**; legacy keeps closed loop. |
| 5 | **Coarse accept band** ("resolution too low") — APPROACH_OK 8×32=256 usteps (~1.1 mm). | **48 usteps (~0.25 mm)** on v2. |
| 6 | **Frame-save bug** — `goto` saved `_last_joints = commanded target`, baking a missed move into `phil_frame.json` (found it at (1024,512) while A1=(0,0) → reconnect shifted the whole frame → A1 off-plate). | Save the **actual read-back**. |

Good config backed up: `software/phil/config/_GOOD_BACKUP/` (24 wells + 0.31 mm kinematics).

## Recovery (no encoder → step-loss is invisible to software)
Jog the outlet onto **A1** center, then `rehome` (or `sethome`). A1 is taught at exactly
(0,0), so this restores the ENTIRE taught frame — **never re-teach**. Only ever home on A1.

## Operational gotchas (these bit us repeatedly)
- **Arrow-key jog is ONLY in `jog_teach`**, not the cli (`phil>` arrows print `^[[A`).
- `cli` flag is **`--backend v2`**; `jog_teach` flag is **`--v2`**.
- Never press `h`/sethome except physically centred on A1.

## TOMORROW — hardware test (start gentle, prove no step-loss first)

```
cd software
# 1. extra-gentle profile for the first proof
PHIL_VMAX=1.0 PHIL_AMAX=8 python3 -m phil.cli --backend v2 --no-check
> travelz 60000
# 2. STEP-LOSS DRIFT TEST (the key check): big move out and back, eyeball A1
> goto H12          # taught far corner — should land on H12
> goto A1           # taught home — should come back DEAD ON A1
#    repeat 2-3x. If A1 stays centred -> no step loss. If it drifts -> lower vmax/accel more.
# 3. if A1 holds, test the MODEL on untaught interior wells:
> sweep B5 C7 D9 E6 F4 G8 B11 F11
#    watch each land; model predicts these (offline LOO ~0.4 mm; ~1 mm backlash floor on top).
# 4. if a well is off, re-home A1 and retry; if a region is consistently off, teach a few
#    wells there (jog_teach --v2 <wells>) and `fitkin` (now warm-started, converges reliably).
```
If after lowering vmax/accel the drift test still fails, the arm may need mechanical
attention (belt tension/binding) — step loss at very low speed is mechanical, not profile.
