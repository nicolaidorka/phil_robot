# Rules — operating & development

## Process discipline — LOG EVERY FAILURE & EVERY COMPLAINT (mandatory)
- **Log every failure mode in [LEARNINGS](LEARNINGS.md).** Anything that broke, was
  reverted, regressed, corrupted data, wasted the user's time, or was just done
  badly — write it down (newest first): what happened, the root cause, and the rule
  it burns in. Do this BEFORE moving on, not "later."
- **Log every piece of NEGATIVE FEEDBACK from the user** in LEARNINGS too — verbatim
  gist + what I did wrong + how to avoid it next time. If the user is frustrated,
  corrects me, or says something is bad/wrong/dumb, that is a learning to record, not
  just a thing to fix in the moment.
- The point is a permanent, cross-session record so the SAME mistake is never made
  twice. When unsure whether something counts — log it.

## KNOW THE HARDWARE FIRST — before ANY precision / accuracy / "it's a bit off" work
**Mandatory.** Before chasing a positioning error by eye, by software, or by
re-teaching, READ what the motors can physically do and state the floor up front.
Do NOT theorize about backlash / speed / step-loss or nudge counts by eye before
this. (Cost the user a long, frustrating session by skipping it — 2026-06-13.)
- **Where it's written:** [`V2-FIRMWARE-NOTES.md`](V2-FIRMWARE-NOTES.md) (motors,
  currents, tip resolution, board) and `software/phil/constants.py` (steps/rev,
  microstepping, geometry, `mm_per_ustep`).
- **The motors (this unit):** Shinano **SST43D1125 NEMA-17**, **200 full-steps/rev**
  (1.8°), v2 = **256 µsteps/full-step** (32× the legacy 8), driven by a **TMC2660 at
  X/Y = 560 mA** off a TMC4361A (Teensy 4.1 / Squid V4). Open-loop, **no encoder**.
- **Resolution at the nozzle:** ~**170–227 µsteps/mm** (5-bar Jacobian; ~120/mm
  measured near H12). So **one full-step at the tip ≈ 1.3–2 mm near H12**.
- **THE FLOOR (why "a bit off" can't be nudged out):** at 560 mA the **microstep
  holding torque can't hold intermediate positions under the heavy 5-bar arm**, so the
  outlet settles toward the nearest full-step detent. Effective repeatability ≈ **one
  full-step ≈ ~1–2 mm**. A sub-full-step count shift moves the tip **nothing** (proven:
  a ~0.5 mm shift did not move it). Chasing below this by eye/software is wasted effort.
- **To actually go below ~1 mm it is HARDWARE, not code:** raise X/Y motor current
  (560 mA is conservative — more current → microsteps hold → finer effective res),
  reduce mechanical backlash (belt/pulley/linkage play), or add an encoder (closed
  loop). **NOT a camera** (user has banned it). State this to the user instead of
  iterating by eye.

## Safety (physical arm)
- **Jog small.** Rotary joints, very sensitive (~50 usteps/well). A few hundred
  usteps crosses the whole plate or runs off the edge. Start with small steps;
  the `jog_teach` console defaults to multiples of 8 usteps (≈1 mm).
- **Lift / clearance before big XY moves** if anything is on the deck. `goto`
  moves XY then sets Z; set a travel Z if you need clearance.
- **Don't blind-fire HOME / limit-switch homing.** It's unverified on this
  firmware and could drive an arm into a hard stop. Use manual `sethome` (zeros
  at the current pose, no motion) instead.
- When commanding motion you can't see, move in small bounded steps and confirm
  with the operator.

## Don'ts
- **Don't hand-move the arms to set a position.** Open-loop, not tracked, and it
  can lose steps. Position only changes via commanded jogs.
- **Don't `reset()` / `initialize_drivers()` mid-session on legacy** unless you
  intend to zero the frame — both wipe the joint counter. (Legacy connect skips
  them on purpose.)
- **Backend defaults to `v2`** (`constants.DEFAULT_BACKEND`) — the flashed
  microstep firmware, matching the v2-scale (ustep_scale=256) teach data. Just run
  `python3 -m phil.cli` (no flag). The board was reflashed 2026-06-13, so **legacy
  is the dead path now**: on legacy the v2 24-byte status mis-frames (garbage
  positions like `33320960`) and the scale guard disables goto. Use `--legacy` /
  `backend="legacy"` ONLY if the firmware is ever rolled back. **Don't use
  `backend="stock"`** — the 8/24 protocol is rejected.
- **Don't re-teach after a power-cycle.** Use `reanchor <well>` (1-well, translation).
  For a sharper edge fix, anchor the 4 corners: `anchor A1/A12/H1/H12` then `anchor fit`
  (affine correction; never refit the 5-bar — that's non-convex and can regress).

## Day-to-day
- Run from `software/`. `python3 -m phil.cli`, then `goto <well>`.
- **Check on start and finish.** A1 is the anchor well. The CLI moves to A1 at
  startup so you can eyeball it, and parks on A1 at shutdown. If the outlet is
  ever NOT on A1 during the check, the frame slipped — jog onto A1 and `reanchor`.
  Run `check` anytime you suspect a bump. (`--no-check` skips the auto-moves.)
- After a **power-cycle or accidental bump**: on connect the robot warns if the
  frame looks reset. Jog the outlet onto A1 and run `reanchor` (defaults to A1).
  Calibration restored; **no re-teach** (geometry in `phil_kinematics.json` is
  permanent). A mid-session bump that skips steps isn't auto-detected (no
  encoder) — that's what the start/finish `check` is for.
- Switch plates: `--labware "<name>"` (list with `labware`); geometry maps the
  new JSON's wells.

## Teaching (the production path — boundary taught, interior interpolated)
- The 5-bar overfits when it has to *extrapolate* to edges (LOO ~1.5–4 mm). Fix:
  **teach the boundary (rows A–E + H, 72/96) and refit**, so the model only
  *interpolates* the untaught interior rows **F and G** (bracketed by taught E
  and H). `goto` is taught-first; F/G come from the refit model (F6 verified
  ~0.5 mm, **column 1 still weak**).
- ⛔ **NOT currently in place (post-reflash, 2026-06-13).** That 72-well boundary
  was taught on the legacy firmware and is **invalid in the v2 frame** (it's in
  `config/pre-reflash-backup/`, not loaded). The live v2 teach is only **24/96**
  (an L-shape), so the model is *extrapolating* today (corner LOO ≈ 12–13 mm).
  **Redo the boundary teach in v2** (`jog_teach --all --v2` → `fitkin`) before
  trusting any untaught well. See [UNITS-AND-CALIBRATION](UNITS-AND-CALIBRATION.md).
- To teach more wells (e.g. push F/G or column-1 below model error):
  `python3 -m phil.jog_teach --all` — snake order, auto-approaching each from the
  last. Nudge to center, Enter to record, `n` to skip, final approach in ONE
  direction (backlash), "over the well" is good enough. **Do NOT press `h`** in
  `--all` — it zeros the frame and wrecks the wells already taught. `s` saves,
  `q` quits — rerun `--all` to resume. ⛔ **Do NOT run `fitkin` after a normal
  teach pass** — it refits the retired, non-convex 5-bar and can REGRESS a good
  calibration; taught wells short-circuit the model anyway and untaught wells use
  the rigid grid. `fitkin` is ONLY for a physical geometry change (next section).
  After teaching, the steps that matter are: set `travelz`, teach `WASTE`.

## Re-fitting the 5-bar (rare — only if the arm geometry physically changes)
1. Do a *fresh* teach of a spread of wells (4 corners, 4 edge midpoints, 2
   middle). On the FIRST well only, center it and press `h` (home) to zero the
   frame — this is the ONLY time homing is correct (it invalidates existing
   taught wells, which a geometry change makes stale anyway).
2. `fitkin` (or it's saved) → refits the 5-bar (in-sample RMS < ~0.5 mm; do not
   expect edge accuracy — it overfits, which is why we teach the dense boundary).
3. Re-teach the boundary rows (`--all`, `n` past the interior) so `goto` replays
   measured joints on the perimeter and the refit model interpolates F/G.

## Development
- Keep `legacy_mc.py` matching the firmware framing (6/20, CRC-8/CCITT, 256
  microstepping). If the Teensy is ever reflashed with the repo firmware, switch
  to `backend="stock"` and the standard `constants.py` units.
- Optional deps: `scipy` (for `well_map.py` and `kinematics.py`). The package
  degrades gracefully if absent (falls back to affine).
- Verify changes in simulation first: `python3 -m phil.cli --simulate` /
  `python3 -m phil.selftest --simulate --move`.
- `constants.py` mirrors `control/_def.py`; the rotary "mm" there is nominal —
  real outlet positions come from the kinematic model, not those constants.
