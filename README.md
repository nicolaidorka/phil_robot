# Phil — 5-bar arm well-plate navigation

Phil is an **articulated 5-bar arm robot** that holds one outlet/nozzle over a
96-well plate. It drives any well of any labware from its JSON definition, using
a fitted **5-bar inverse-kinematic model** — no per-well teaching once calibrated.

> Phil is **not** a microscope. It reuses the Squid/octopi codebase only for the
> Teensy motor firmware; the control software here is the self-contained `phil`
> package. See [`software/phil/README.md`](software/phil/README.md) for the
> module reference and [`CLAUDE.md`](CLAUDE.md) / [`.claude/`](.claude/) for
> operating notes and the mechanism description.

## How it works

- **X and Y are rotary arm joints** (two base motors, each driving a link; the
  two links meet at the outlet). **Z** is vertical. Open-loop steppers, no
  encoders, ~1–2 mm precision floor.
- A handful of taught wells fit the arm's geometry (link lengths, pivots) to
  ~0.2 mm RMS; inverse kinematics then computes joints for **any** well, at the
  edges too. Resolution order: kinematics → RBF map → exact taught → affine.
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
  geometry/       well_plate, calibration, kinematics (the solver), well_map
  hardware/       legacy_mc (Teensy 6/20-byte firmware driver)
  teaching/       teach (table), jog_teach (arrow-key console)
  cli.py          interactive shell
  selftest.py     hardware self-test
  config/ labware/ custom_labware/    JSON state + plate definitions
tests/            sim-backend smoke tests  (pytest)
```

## Tests

```bash
pytest -q          # sim backend only; no hardware required
```

## Acknowledgement

Built on the [Squid](https://github.com/hongquanli/octopi-research) firmware/
controller stack.
