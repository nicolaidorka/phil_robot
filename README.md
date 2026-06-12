# Phil — 5-bar arm well-plate navigation

Phil is an **articulated 5-bar arm robot** that holds one outlet/nozzle over a
96-well plate. It drives wells of any labware from its JSON definition. Because
the open-loop arm has backlash, the dependable path is to **teach each well and
replay its exact joints**; a fitted 5-bar inverse-kinematic model fills in any
well not yet taught (it overfits at the edges, so a taught well always wins).

> Phil is **not** a microscope. It reuses the Squid/octopi codebase only for the
> Teensy motor firmware; the control software here is the self-contained `phil`
> package. See [`software/phil/README.md`](software/phil/README.md) for the
> module reference and [`CLAUDE.md`](CLAUDE.md) / [`.claude/`](.claude/) for
> operating notes and the mechanism description.

## How it works

- **X and Y are rotary arm joints** (two base motors, each driving a link; the
  two links meet at the outlet). **Z** is vertical. Open-loop steppers, no
  encoders, ~1–2 mm precision floor.
- **Teach each well, replay exact joints.** `goto` returns the taught joints for
  a well when it has them (72/96 taught so far; teach the rest with
  `phil-teach --all`). A 5-bar model fit from taught wells (~0.2 mm in-sample
  RMS) and an RBF map are fallbacks for untaught wells — the model generalizes
  to any labware but overfits at the edges, so resolution is **exact taught →
  kinematics → RBF map → affine**.
- Any Opentrons-style labware JSON works — well mm-coordinates flow through the
  same geometry.

## Install

```bash
pip install -e .          # editable install (recommended; keeps config/ writable)
```

This exposes console scripts `phil`, `phil-teach`, `phil-selftest`. You can also
run in place from the `software/` directory without installing:

```bash
cd software
python -m phil.cli                 # interactive control (real hardware)
python -m phil.cli --simulate      # no hardware
python -m phil.selftest --move     # connection + feedback + tiny jog
python -m phil.jog_teach           # guided teach console
```

Dependencies: `numpy`, `scipy`, `pyserial` (see `requirements.txt`). scipy is
optional at import time — the sim backend and affine fallback work without it.
Real hardware backends (`legacy`/`stock`) additionally need the Squid `control`
package on `sys.path`.

## Usage

```python
from phil import PhilRobot
bot = PhilRobot(backend="legacy")   # legacy = this Teensy's older firmware
bot.connect()
bot.goto_well("B10")
bot.close()
```

## Package layout

```
software/phil/
  robot.py        PhilRobot: connect, jog, goto_well, teach, reanchor  (core)
  paths.py        single source of truth for config/labware locations
  constants.py    stepper geometry + motion defaults
  cli.py  jog_teach.py  selftest.py   the 3 entry points (python -m phil.<name>)
  geometry/       well_plate, teach, calibration, kinematics (the solver), well_map
  hardware/       legacy_mc (Teensy 6/20-byte firmware driver)
  config/         runtime JSON state (teach/calibration/kinematics)
  labware/        all plate definitions (Opentrons-schema JSON)
tests/            sim-backend smoke tests  (pytest)
```

## Tests

```bash
pytest -q          # sim backend only; no hardware required
```

## Acknowledgement

Built on the [Squid](https://github.com/hongquanli/octopi-research) firmware/
controller stack.
