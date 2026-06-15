# Learnings — open-loop arm kinematics, frames, and calibration (Phil)

Hard-won lessons from debugging Phil (a 5-bar parallel-arm, open-loop steppers, no encoders, v2
microstep firmware). Written so this class of problem isn't re-derived. Read before touching the
goto / frame / kinematics path.

---

## 1. Open-loop frame management — the counter is the only truth, and it lies easily

There are **no encoders**. The robot's *only* notion of position is the firmware's joint counter
(TMC4361A `XACTUAL`), which is **volatile** (lost on power/INITIALIZE) and **blind to physical
reality** (it tracks commanded steps, not where the arm actually is). Everything below follows from this.

- **The counter must be persisted on the host and restored on connect** (`phil_frame.json`). On v2,
  connect does INITIALIZE (zeros the counter) → `set_position_usteps(last)` to restore.
- **ALWAYS confirm a fire-and-forget counter write.** `set_position_usteps` has `expect_ack=False` — no
  COMPLETED comes back. If you `set_position` then immediately `joint_position()`, you read a **stale**
  packet (the reader thread still holds the pre-write value) and bake a wrong number into the frame
  file. This silently drifted the whole frame ~12 mm across reconnects. Fix: poll the live stream
  (firmware streams position every ~10 ms regardless of motion) until the counter reads the target,
  with **2 consecutive matching reads**, and **WARN + refuse to save on timeout** — never persist a
  frame you couldn't confirm. (`robot._confirm_counter`, used in `set_home` AND the connect-restore.)
- **Persist the SETTLED actual read, never the commanded target or a single read.** A `goto` that lost
  steps, or read one stale packet, would otherwise write a wrong "last pose."
- **Step loss is invisible.** The counter follows commands, so a stalled/skipped motor reads
  "on-target." You cannot detect it in software — only by eye. Mitigate by *not losing steps* (gentle
  profile, no jerk) and by re-anchoring against a known well.

## 2. Anchor / home corruption — never let one point do triple duty

Phil used **A1 = joints (0,0) = home = anchor = a taught well**. That coupling is a trap: corrupting
A1 (e.g. an accidental Enter that recorded it off-center, then homing there) **poisons the entire
frame**, because every well is referenced to that zero. And A1 being the (0,0) origin makes it the one
point *immune to a multiplicative scale error* — so a 32× unit bug masquerades as "A1 is the only well
that's right." Lessons:

- **Anchor to the CONSENSUS grid, not a single point.** The body of consistent wells + the kinematic
  model define the frame robustly; one bad well can't shift it. Prefer a multi-well anchor
  (`anchor`/`anchor fit` over several good corners) over a single `reanchor`.
- **Validate an anchor before trusting it.** `grid_check()` maps taught joints → mm via `forward()`
  and flags wells off the uniform 9 mm grid. **Refuse to anchor on a flagged well** (we repeatedly
  re-anchored on the corrupt A1 — exactly the mistake the gate prevents).
- **A single-well `reanchor` is a pure TRANSLATION.** It can fix a frame *shift*; it cannot fix a
  *scale* error or a rotation. If your error grows with distance, a translation anchor is the wrong tool.

## 3. Coordinate conventions — a kinematic fit absorbs RIGID transforms; don't chase orientation

The plate is physically rotated 90° (A1 upper-right), while the labware JSON is a standard plate. This
looks alarming but is a non-issue: **a 5-bar fit absorbs any rigid rotation+translation+scale** because
it learns the joint↔mm map from the taught points. We *proved* it: the fitted model's column-axis and
row-axis came out 88.7° apart with near-equal magnitude (a clean orthogonal grid), and every taught
well reproduced to ~1.3 mm.

- **Before blaming orientation, test whether the residual is RIGID (noise) or NON-RIGID (transpose/
  reflection).** Fit a similarity/affine from JSON-mm → joints; if the linear part is ≈ identity-up-to-
  rotation with small residual, orientation is absorbed — the bug is elsewhere (frame/anchor/scale). A
  true transpose (rows↔cols swapped) the fit *cannot* absorb and shows as large structured residuals.
- The labware JSON's job is only to give *consistent* mm coordinates; the rotation handling is the
  fit's job. Don't add ad-hoc transposes to `local_xy`.

## 4. Unit-scale mismatches — the silent 32×

Legacy firmware reports/commands at ~8 usteps/full-step; v2 microstep firmware at 256 (32× finer). The
teach/kinematics files carry a `ustep_scale` stamp. **A mismatch between the data's scale, the
`--backend`, and the firmware on the Teensy is catastrophic and silent** (every well 32× off, growing
with distance — except (0,0)). The existing guard only protects v2-from-legacy-data; make it
**symmetric** (legacy backend must refuse v2-scale data). First diagnostic on any "everything's off"
session: **jog a known amount and read the count delta** — it tells you which firmware/scale is live.
Constant offset ⇒ frame/anchor; offset growing with distance ⇒ scale.

## 5. Diagnostics that localize the fault (model vs frame vs orientation vs scale)

- **`forward(taught_joints)` vs JSON-mm** per well → which wells don't fit the grid (flags mis-taught).
- **Leave-one-out**: drop a taught well, refit, predict it. Sub-mm LOO ⇒ model is good, error is FRAME.
  Large/structured LOO ⇒ model/geometry is wrong.
- **The measure tool** (drive via model → user centers → record offset): a CONSTANT offset (small
  spread) ⇒ one frame translation fixes all (reanchor); PER-WELL spread ⇒ teach those wells. This one
  table separates "frame" from "model" definitively.
- **`gridcheck`**: are predicted/taught centers a uniform grid? Confirms the error is a pure translation.
- **`jacobian_sign` constancy**: a flip means the fit crossed into the mirror assembly mode.

## 6. 5-bar kinematics specifics

- **The fit is non-convex** (a near-symmetric 5-bar has degenerate/mirror minima). A single random seed
  finds the true geometry only ~1/3 of the time; a bad draw shows as obviously high RMS. Use **best-of-N
  seeds + a warm-start from the known geometry** (proximal 65 mm, distal 145 mm — the published PHIL
  values) and a **soft prior** pulling link lengths toward them (blocks divergence to giant-link
  solutions) — but anchor the *origin* with real data, NOT a median-of-taught (that's self-defeating;
  see §7).
- **Assembly-mode mirror ambiguity**: forward kinematics is a circle-circle intersection (two modes);
  inverse has 4 working modes. Pin the branch and guard every prediction with a constant `sign(det J)`.
- **`_unwrap` must be scale-independent** (unwrap toward the taught-joint center, not a hardcoded
  window) or v2's ±thousands joints wrap to garbage.
- **Notional velocity units**: `set_max_velocity_acceleration`'s "mm" are multiplied by ~12 800
  usteps/"mm" in def_phil.h — so "vel=4" is ~250 mm/s at the tip. v2 *honors* vel/accel (legacy ignored
  it), so an aggressive profile **skips steps**. Keep accel especially low; one continuous smooth MOVETO
  beats chunked stop/start legs (each restart is a jerk and a skip chance).

## 7. Process lessons (how we wasted time, and how the right answer came)

- **Adversarial review caught a self-defeating fix.** I proposed anchoring the model origin with
  `median(taught − inverse(mm))`. But the taught wells *define* the model's zero-offset frame, so that
  median is ≈0 and fixes nothing. The correct anchor comes from a CURRENT-frame measurement (live joints
  at a known well). **Always test a calibration fix against held-out ground truth, not the fit data.**
- **Empirical beats theory.** Hours of "is it the model / frame / orientation?" were settled in minutes
  by running LOO and overlaying `forward(taught)` vs JSON on real numbers.
- **Stop patching; find the root cause.** Each one-off frame patch got re-corrupted on the next
  reconnect because the underlying confirm-the-write bug was unaddressed. Fix the mechanism, not the
  symptom.
- **Two hypotheses that make OPPOSITE predictions are a gift** — design the one cheap test that splits
  them (constant vs distance-growing offset) instead of arguing.
