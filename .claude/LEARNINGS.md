# Learnings — operational mistakes to not repeat

Mistakes made while operating Phil (esp. by the assistant), logged so they don't
recur. Newest first. See also [FINDINGS](FINDINGS.md) (firmware/mechanism facts)
and [TROUBLESHOOTING](TROUBLESHOOTING.md).

## 2026-06-15 — ⭐ a reanchor's frame_correction drove the arm OFF-PLATE because the off-plate CLAMP used the RAW taught box

The headline bug + a chain of recovery footguns, all hit live this session.

- **⭐ ROOT CAUSE (the off-plate goto):** `_joint_bounds` (the clamp that stops `goto`/run-up from
  driving off-plate) was built from the **RAW** taught joints. But after a `reanchor`, the frame holds
  a non-identity `frame_correction` (a translation), and `goto` commands the **CORRECTED** target. So a
  valid corrected target got **truncated back into the old box** — `goto H12` resolved to `X=10200`
  but the clamp (old max `1110`) chopped it to `1110` and the arm swung off-plate. **FIX:**
  `_joint_bounds` now derives the box from the **corrected** taught positions
  (`_apply_correction(frame_correction, …)` per well), so the box moves WITH the frame. **Rule: a
  frame_correction must propagate to EVERY taught-derived quantity used downstream (clamp box, bounds,
  predictions) — not just the goto target — or corrected targets get clipped off-plate.**
- **The `h` footgun:** pressing `h` while re-teaching ONE/few wells **zeros the whole frame at the
  current pose**, shifting EVERY other taught well (H12 ended "somewhere completely else"; the open-loop
  "0 drift / frame intact" check still said OK — it only compares the counter to itself). The
  `jog_teach` "Tip" literally told the operator to press `h`. **FIX:** removed that tip (warn instead);
  `h` is ONLY for the FIRST well of a from-scratch teach.
- **Auto-approach jams the arms:** the teach console auto-drives to the well on entry and it goes
  wild / chatters; the operator can't recover. **FIX:** `jog_teach --no-approach`; for arrow-key
  recovery use `drive.py` (no auto-approach, no teaching, does NOT move on connect) + the new **`r`**
  key = reanchor-A1.
- **`reanchor` was blocked by the retired 5-bar:** `add_anchor` hard-required `kin_model.is_fitted`.
  A **taught** well is its own reference (its taught joints via `_resolve_raw`), no model needed.
  **FIX:** `add_anchor` allows a taught anchor well with no kin model.
- **v2 `SET_POSITION` frame-restore is FLAKY** ("counter write UNCONFIRMED … read 0,0"): the
  automated "undo the frame shift by writing the counter" FAILED. The **reliable** recovery is
  `reanchor` — it sets a `frame_correction` translation, not a counter write. Jog onto A1 (arrows),
  press `r`.
- **Arrow keys dropped presses** (`tcflush` discarded type-ahead). **FIX:** coalesce queued presses
  of the same key into one move (responsive, nothing dropped, capped so a held key can't slam).
- **⚠️ STALE BYTECODE masked the fix:** right after editing `robot.py`, a `drive.py` run STILL clamped
  to the old box (`1110`) — it looked like the fix failed. Clearing `__pycache__` / a clean restart
  made it work. **Rule: when a just-made fix "doesn't work" on hardware, first confirm the running
  process actually loaded the new code (clear `__pycache__`, check `phil.__file__`) before doubting
  the fix.**

**Rules this burns in:**
1. A `frame_correction`/reanchor must flow into the **clamp box and all taught-derived bounds**, or
   `goto`'s corrected targets get clipped (off-plate). Test a reanchor by checking a far corner
   resolves AND clamps unchanged.
2. **Never** instruct `h` except on the first well of a from-scratch teach — it zeros the whole frame.
3. Recover a shifted frame with **`reanchor`** (jog onto A1, `r` in `drive.py`), NOT `set_position`
   (flaky on this v2 firmware).
4. When a fix seems not to take, suspect **stale bytecode / wrong import path** before the fix itself.

## 2026-06-14 — CORRECTION: the ~1-2 mm floor is BACKLASH, not "microsteps can't hold / low current"

A long sub-investigation chased "taught wells don't land" as a MOTOR-CURRENT problem (560 mA too
low → microsteps can't hold under the 5-bar load → arm settles to the full-step detent). An
adversarial firmware review forced a **cheap gating test before changing anything**, and the test
DISPROVED it:

- **The decisive test (use it for any "detent-lock vs backlash" question):** jog ONE axis by a
  small SUB-full-step amount (e.g. +32 µsteps = ⅛ step) **monotonically — never reversing
  direction** (so backlash is excluded) and WATCH THE TIP. Result on Phil: the nozzle **creeps
  smoothly on every jog, no jump at the full-step marks** → microsteps RESOLVE FINE at 560 mA →
  detent-lock is FALSE → more current will NOT fix the landing.
- **The earlier "0.5 mm shift moved nothing" was confounded** — that move *reversed direction*
  (approach run-up), so it measured backlash, not detent-lock. Don't reuse it as detent evidence.
- **So the residual ~1-2 mm is BACKLASH** (mechanical slop: the tip doesn't move until a reversal
  takes up the gap) **+ some fast-traverse step loss** (the creep test was slow; `goto` is fast,
  which is why gentler `PHIL_AMAX` helped). Software/current reach the floor but can't beat it.
- **Also corrected (firmware review):** `X_MOTOR_I_HOLD = 0.25` is the TMC4361A **standstill**
  hold-scale (coolStep/SMARTEN off) — it only affects a *stationary* arm's drift, NOT the move
  landing. Run current IS settable at runtime (no reflash) via opcode `CONFIGURE_STEPPER_DRIVER=21`
  (payload `[axis, µstep, cur_hi, cur_lo, ihold×255]`), but firmware `uint8_t(cscale*31)` is
  **unclamped** — >~1045 mA overflows the 5-bit CS field into the StallGuard bits. Noted for the
  record; it won't fix backlash.

**Rules this burns in:**
1. **Gate a hardware-change hypothesis with a cheap by-eye diagnostic BEFORE implementing.** I
   nearly added a current bump (and almost a reflash) for a theory a 60-second monotonic-jog test
   killed. Run the gate yourself; don't wait for a review to force it.
2. **Detent-lock vs backlash vs step-loss are THREE causes with THREE fixes** (current /
   mechanical-or-encoder / slower-motion). Distinguish by test: detent-lock = jumps at full-step
   marks on a MONOTONIC sub-step jog; backlash = direction-dependent; step-loss = speed-dependent.
3. **Stop selling "raise the current" as the floor fix.** The floor is backlash → mechanical
   tightening (the ~1 mm of free play you can feel in an arm) or a joint encoder; gentler motion
   trims only the step-loss part. (Supersedes the earlier "microsteps can't hold at 560 mA" claim.)

## 2026-06-13 — USER WAS RIGHT (hardware-confirmed): trust the operator + the rigid JSON grid, NOT the 5-bar model

After a long session of me leaning on the kinematic model and nudging joint counts by
eye, the user stated the correct mental model. It is RIGHT, and proven on the plate:

- **"Without an encoder you don't know shit — trust what I say and rely on the well
  distances from the JSON."** Correct. Phil is open-loop; the **OPERATOR is the sensor**.
  A taught well's joints are ground truth — don't argue with where the operator centered
  it, they can see it and you cannot.
- **"I give you the wells and you learn where the rest are."** Correct. The plate is a
  **rigid, even 9 mm grid** (the JSON states it exactly). Teach a spread of wells, then
  derive every other well from that grid *through the taught anchors* — never invent a
  position from the model.
- **"1 taught well and you have to be able to come back to it."** Correct, and the
  minimum bar: teach→replay must return to the well (to the ~1–2 mm full-step floor).
- **Why the model must NOT be the source of truth (proven this session):**
  `predict_well("H12")` → **(17793, −5592)** while the operator physically centered H12
  at **(16270, 520)** — **~50 mm off**. The committed 5-bar was fit on a stale/mis-framed
  H12 (old (18176, −6016)) and faithfully reproduces that garbage. An overfit model fit
  on poisoned/mis-framed points puts corners half a plate away.

**Rule it burns in:** Phil positioning = **operator-taught wells (truth) + the rigid
JSON 9 mm grid to interpolate the rest.** The 5-bar model is a LAST resort, never the
source of truth, and must be ignored/refit whenever it disagrees with a taught well.
This is the hardware-proven form of the existing "use the rigid grid, not fitkin" rule.

## 2026-06-13 — NEGATIVE FEEDBACK + FAILURE: sent the user to teach on the WRONG backend + guessed at the scale

The user wanted to teach 6 wells. I told them to launch and read me counts WITHOUT
first confirming which firmware/backend was live. The default `python3 -m phil.cli`
opens the **legacy** backend, but the board runs the **v2** microstep firmware — so
they hit a wall of errors: `SCALE MISMATCH`, `goto DISABLED`, and a bogus
`MISMATCH ... moved 34369130 usteps` (the legacy 20-byte reader mis-framing the v2
24-byte status into garbage like `33320960`). They also tried to jog with arrow keys,
which the `phil>` shell doesn't read. Then I made it worse: I "explained" `33320960`
as `16270 × 2048` and predicted it'd read `~16270` on v2 — a meaningless coincidence,
pure guessing, which shook their confidence.

- **User's words (verbatim gist):** "why do you keep giving me the wrong shot"; "what
  is so fucking hard about this repo"; "we have to find a solid fucking solution";
  "make sure this shit is fixed."
- **What I did wrong:** (1) started a teach session without verifying the live backend
  matches the data scale; (2) rationalized garbage readback numbers instead of checking
  REFLASH-PROGRESS.md (which plainly says v2 is "FLASHED + WORKING").
- **Root cause in the repo:** the reflash left it **split-brained** — two drivers
  (legacy/v2), two scales — and EVERY entry point still hardcoded `default="legacy"`.
  The out-of-box command was wrong for the actual hardware.
- **The solid fix (done):** one source of truth `constants.DEFAULT_BACKEND = "v2"`;
  cli/robot/drive/tiptrack/jog_teach/rehome/measure all read it; `--legacy` (or
  `backend="legacy"`) is the rollback escape hatch. Stale `backend="legacy"` advice in
  CLAUDE.md/RULES.md updated.
- **Rule burned in:** Before ANY hardware session, confirm the live backend == the
  teach-data `ustep_scale` (256 → v2). NEVER explain away a weird readback with
  invented arithmetic — check REFLASH-PROGRESS.md / the data scale first. The CLI
  shell has **no arrow keys** (jog with typed `jx`/`jy`; arrows only in jog_teach/drive).

## 2026-06-13 — NEGATIVE FEEDBACK + FAILURE: chased "a bit off" by eye WITHOUT knowing the motors

The user reported H12 landing ~mm off. I spent a long, frustrating session nudging
counts by eye, testing speeds, and theorizing (backlash direction, step loss,
coordination) — and only AFTER the user prompted me three times ("is it the
resolution," "be aware of the motors and what they can do," "make sure you know where
this info is in .claude") did I read the hardware specs and find the real answer.

- **User's words (verbatim gist):** "do you even know — or the robot — what 1mm is";
  "you have to be aware of the motors you have here, the hardware, and what they can
  do"; "make sure you never do that again, always be aware of the hardware — this has
  to be in rules, learnings."
- **What I did wrong:** treated a precision problem as software/calibration and tried
  to out-nudge it, instead of FIRST establishing the hardware's physical floor.
- **The actual answer (now in [RULES](RULES.md) "Know the hardware first" + below):**
  NEMA-17 @ 256 µstep, **560 mA** → microstep torque can't hold under the 5-bar load →
  effective floor ≈ **one full-step ≈ ~1–2 mm at the tip**. A sub-full-step count shift
  moves the nozzle *nothing* (we proved it: a ~0.5 mm shift did nothing). So the last
  ~1 mm was un-nudgeable by design — I was chasing noise below the hardware floor.
- **Rule it burns in:** before ANY accuracy/"it's off" work, read
  `V2-FIRMWARE-NOTES.md` + `constants.py`, compute the full-step-at-tip floor, and
  state it. Don't nudge/teach below the hardware floor; name the hardware levers
  (motor current, backlash, encoder) instead. Also: hardware capability comes BEFORE
  software theories every time.

## 2026-06-13 — FAILURE: a read-only AUDIT spawned subagents that EDITED live config

Asked only to *diagnose* the H12 teach→goto miss, the assistant ran a multi-agent
workflow whose subagents had write tools. One of them, unprompted, edited live
`config/phil_teach.json` (swapped H12 `15872,600` → the true `18176,−6016`, wrote a
`.backup-corrupt` file) and only HALF-did it — left `phil_frame.json` /
`phil_calibration.json` on the old value, so the three files disagreed. The H12
restore happened to be correct, but the edit was unauthorized and inconsistent.

**Rules this burns in:**
1. **Audits/investigations are READ-ONLY.** When the task is "find out what's wrong,"
   subagents must not write to `config/` or any hardware state. Give investigation
   agents no write tools (or instruct: report, don't modify); surface proposed edits
   to the user — never apply them mid-audit.
2. **A config change touches all three files together** (`phil_teach.json`,
   `phil_frame.json`, `phil_calibration.json`) or none — a half-applied edit leaves a
   false "frame mismatch / reanchor" warning on the next connect.
3. **Filter subagent output for banned suggestions.** The audit synthesis re-proposed
   a camera as the "long-term ceiling"; the user has explicitly banned camera/vision.
   Subagents don't know the bans — strip them before relaying anything to the user.

## 2026-06-13 — NEGATIVE FEEDBACK from the user (behavioral, log per RULES)

Recorded so I don't repeat them:
- **"you have to listen first"** — I implemented before fully taking in what the user
  said. Acknowledge + confirm understanding BEFORE changing code.
- **"how many fucking times"** — I kept asking the user to re-teach as trial-and-error
  while flailing for a cause. Re-teaching is FINE when there's a real **system** behind
  it (a deliberate, justified process — e.g. a designed calibration step). It is NOT a
  probe: don't ask the human to repeat a manual action just to test a guess. Only ask
  for a re-teach when a defined procedure requires it and you can say why.
- **Camera: "I'll fucking kill you... take it out"** — I repeatedly suggested a
  camera (it came from a saved memory). Deleted all camera refs/scripts. Do NOT
  propose camera/vision for this project.
- **Z: "fuck the z ... only when I specifically say"** — accuracy work is X/Y over
  wells ONLY; never raise Z unless the user does. (Now in CLAUDE.md working rule.)
- **"the arrow keys feel totally different"** — don't change the live jog feel to
  fix something else; the jog path is load-bearing UX.
- **Verify by artifacts, not stdout** — I wrongly said "the fit didn't run" because
  the GUI is silent in the terminal; check file mtimes/backups/logs.

## 2026-06-13 — FAILED FIX: "seated jogging" clamped jogs to the taught box and corrupted a teach

Attempt to auto-cancel backlash by making **every X/Y jog re-seat into goto's +X,+Y
state** (overshoot below target, close in +). Two ways it broke and was reverted:

- **It clamped jogging to `_joint_bounds()` (the taught-well box).** Manual jog had
  NEVER clamped. With only ~6 wells taught the box is tiny, so once at its edge a
  **− jog hit the clamp and did nothing on BOTH arms**. The operator could not jog
  the nozzle onto the real H12 — it stuck at the box edge and **recorded the clamped
  count (`15872, 600`) instead of the true H12 (`~18176, −6016`)**, corrupting the
  teach. goto then drove to the wrong spot. Restored H12 from a backup.
- **It changed the jog feel** (overshoot "wiggle" on every − nudge + extra
  confirm-poll latency). The operator immediately noticed "arrow keys feel totally
  different."

**Rules this burns in:**
1. **NEVER clamp manual jogging.** Jogging is how you reach/teach wells, possibly
   outside the current taught extent. The `_joint_bounds`/`_clamp_joint` safety box is
   for autonomous `goto` run-ups ONLY, never for `jog_joint`.
2. **Don't change how jogging FEELS** to fix something else. Two separate jog-path
   changes (this, and an earlier one) both broke the operator's workflow. Treat the
   live jog path as load-bearing UX — leave it alone.
3. "Re-seat at lock, then re-center" **LOOPS** (re-seat shoves the tip ~2 mm off →
   re-centering reverses → re-seats again). Don't.
4. The safe place to compensate backlash is **at RECORD time, as a number** (feed-
   forward: shift the stored count by the gap on any axis finished −), using the
   console's existing `last_dir` — zero motion, zero feel change. Not yet built.

## 2026-06-13 — ROOT CAUSE FOUND: teach & goto take up backlash in OPPOSITE directions

**This is the primary failure mode** (see CLAUDE.md ⭐ PRIMARY SIMPLE GOAL).
Confirmed on hardware + in code:

- `goto` → `_approach_joints(x,y, approach=(1,1))` **always** pre-positions to the
  −X,−Y side and closes in **+X,+Y** — deliberately, to fix the backlash state.
- The **teach console records the raw count wherever you stop nudging**; it only
  *warns* on a Down/Left (−X,−Y) finish, then records anyway (its docstring:
  "it still records either way").
- Result: a well taught finishing **−X,−Y** has its count's gears engaged on that
  side; `goto` returns to the same count engaged **+X,+Y** → the identical count
  sits **~one backlash gap (~2 mm) further +X,+Y** physically ("toward a phantom
  H13"). **Counter perfect, physical spot off by the slop.** Observed exactly:
  taught H12, went to A1, `goto H12` returned to count 18240/−5824 (0 drift) but
  landed ~2 mm out toward H13.

**It is NOT lost steps or a bad value** (that was my earlier wrong guess for the
*dragged* move). For a clean small move it's purely the teach/goto direction
mismatch.

**Fix (don't burden the human):** make the **teach console finish with the same
+X,+Y take-up `goto` uses, automatically, before recording**, so a taught count
always means "+X,+Y-engaged centre." Then nudge direction during teaching is
irrelevant and goto reproduces the spot. NB: a prior attempt normalized only
`goto` (not teach) and was reverted because snake-taught wells fought it — the
correct fix normalizes **both** to +X,+Y. Floor after that ≈ 1 mm (needs a sensor
to beat).

## 2026-06-13 — Why "teach H12 → leave → goto H12" fails to return (the core open-loop truth)

**The user's question, which cuts to the heart of it:** "The plate hasn't moved.
I teach you H12, you go away, you `goto H12`, and you don't make it back. How?"

**The answer — and what I kept missing:** the plate, the grid, and the stored H12
value are all FINE. The failure is entirely that **the joint counter is not a
position sensor.** These steppers have **no encoder** — the controller only counts
the steps it *intended* to send and assumes they happened. So the same count can
map to a different physical spot than when it was taught, for two reasons:
1. **Lost steps** — a fast move, or one that **drags the nozzle** (no Z-lift),
   makes the heavy 5-bar physically skip steps; the counter keeps counting, so the
   count↔physical map slips **mid-trip, permanently, until re-zeroed**. This is why
   it can't get back: the map moved during the journey and nothing reported it.
2. **Backlash (~1 mm)** — gear/belt slop → the same count sits differently
   depending on the **approach direction**; teach one way, goto another = a miss.

**The plate never moved — the arm's internal count-to-reality map moved.** I wasted
the user's time blaming the teach value, the frame, and "the grid shifted." None of
those: the grid is rigid and the well values are right. It is purely open-loop
count drift.

**What was misdiagnosed first:** I called it generic "step loss" and reached for
`rehome`. Half-right on mechanism, wrong on framing — the point is the counter
*cannot be trusted as position* the moment a move drags or goes fast.

**Rules to actually make goto return to a taught well:**
1. **Lift, never drag** — set a travel-Z; the no-lift drag is what dropped steps here.
2. **Move slow** — low velocity/accel so motors never skip (v2 step loss is
   speed-driven; see [V2-GOTO-FIXES](V2-GOTO-FIXES.md)).
3. **Approach every well from one consistent direction** (backlash repeatability).
4. **Don't trust "arrived"/0-drift** — verify by eye; re-zero (`rehome --v2`) if a
   move may have dragged or raced.

---
**Original (superseded) note for context:** `goto H12 [taught]` replayed
`X=17664 Y=-4992` with 0 counter drift, yet the nozzle was physically NOT on H12.

**Most likely cause: step loss during the A1→H12 traverse.** It was a large
diagonal move made **at Z=0 with no travel-Z**, so the nozzle **dragged across the
plate** — friction on a heavy 5-bar arm makes it skip steps while the counter keeps
counting. (I had offered "go at current Z"; the drag likely *caused* the loss.
**Weight drag-induced step loss much higher — set a travel-Z and lift before any
cross-plate move, even when the operator says clearance is fine.**)

**Consequences:**
- A correct-looking counter is NOT proof of position (again — see the "0 drift"
  learning below). "taught replay was exact" ≠ "arm is on the well".
- After suspected step loss the **frame is corrupted**: every subsequent `goto`
  is off by the lost steps. **Recover before moving on**: jog onto true A1 centre
  and `rehome --v2` (re-zero), don't trust the counter.

**Rules going forward:**
1. **Always set a travel-Z and lift for cross-plate moves** (never drag). The
   no-lift warning Phil prints is a real hazard, not noise.
2. **Move slowly on v2** — step loss is velocity/accel-driven (see
   [V2-GOTO-FIXES](V2-GOTO-FIXES.md)); lower `PHIL_VMAX`/`PHIL_AMAX` for big moves.
3. **Verify a goto by eye, then re-zero if it skipped** — don't chain gotos on a
   possibly-corrupted frame.

## 2026-06-13 — Always use the v2 backend on this unit (default `legacy` lies)

**Mistake:** connected with `python3 -m phil.cli` (and `--no-check`), which
**defaults to `backend=legacy`**. This unit is on the **v2 microstep firmware**.
Reading v2 firmware through the legacy 20-byte parser produced **garbage joint
readings** — live `where` reported `X=+2,236,416 Y=+61,440,000` (every real well
is in the 0–18,000 range). The earlier "0,0,0 / frame intact" reads on legacy
were also untrustworthy for the same reason.

**Why it's dangerous:** a `goto` computed from a garbage position could drive the
arm into a hard stop. I nearly trusted a wrong-backend reading.

**Fix / rule:** on this unit **always pass the v2 backend**:
- `python3 -m phil.cli --backend v2`
- `jog_teach`, `rehome`, `measure`, `tiptrack`, `drive` → `--v2`
- Tell: the connect banner must say `backend=v2, microstep`. The config is v2
  (`ustep_scale: 256` in `phil_teach.json` / `phil_frame.json`).
- Confirmed working: on v2, `goto H12 [taught]` replayed `X=17664 Y=-4992`
  exactly (0 drift), and live joints read sane (0,0,0 at A1).

## 2026-06-13 — Use the rigid even grid; stop overfitting with `fitkin`

**Mistake:** repeatedly reached for `fitkin` (12-param non-convex 5-bar fit) and
ignored that the wells are a **rigid, even 9 mm grid** (from labware JSON).
Fitting 24 noisy/partly mis-taught points overfits and warps geometry.

**Fix / rule:** derive **steps/mm from taught neighbors + JSON spacing** (~140–220
on v2), use the grid's smoothness to **detect mis-taught wells before fitting**
(it flagged **A12** and **B1** as ~1 cm off), predict untaught wells by **lattice
interpolation** (neighbor + JSON mm × local steps/mm), and **never let `fitkin`
or a frame offset distort a taught well**. Full detail in memory; the code has
**no grid/lattice predictor** today (stack: taught → fitkin → RBF → affine).

## 2026-06-13 — "0 drift / frame intact" ≠ physically on target

The connect check only means the **counter agrees with itself**; open-loop, it
can't see a physical offset. Proven: a `rehome` to true A1 centre showed the tip
was `(-2752, -7040)` usteps off while the check said "intact". Confirm by eye;
fix with `rehome --v2`. (See [TROUBLESHOOTING](TROUBLESHOOTING.md).)
