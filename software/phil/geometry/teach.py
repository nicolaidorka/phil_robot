"""Teach-and-replay well table for the articulated (5-bar) Phil arm.

Because Phil's X/Y are rotary joints on a parallel linkage (not a Cartesian
stage), the safe, measurement-free way to reach a well is to *teach* it: jog
the outlet over the well once and save the joint positions (X, Y, Z motor
positions, in repo microsteps). ``goto`` then replays them.

Untaught wells are estimated by bilinear interpolation in JOINT space from
the four taught corners (A1, A12, H1, H12). That is approximate for a 5-bar
mid-plate, but gives a usable starting point you can refine by teaching more
wells (a taught well always overrides the interpolation).
"""
from __future__ import annotations

import json
import os

from ..paths import DEFAULT_TEACH_PATH  # re-exported so callers keep importing it from here

CORNERS = ("A1", "A12", "H1", "H12")    # default (96-well) corners, for display/messages


def plate_corners(plate) -> tuple:
    """The four geometric corner wells for THIS plate (A1/A12/H1/H12 for a 96,
    A1/A24/P1/P24 for a 384). Use this instead of the hardcoded 96-well CORNERS so
    interpolation/anchor span the real plate."""
    r0, r1 = plate.rows[0], plate.rows[-1]
    c0, c1 = plate.columns[0], plate.columns[-1]
    return (f"{r0}{c0}", f"{r0}{c1}", f"{r1}{c0}", f"{r1}{c1}")


class TeachTable:
    def __init__(self, labware="eppendorf_twintec_lobind_96_pcr.json",
                 z_travel_usteps=None, ustep_scale=None):
        self.labware = labware
        self.taught: dict[str, dict] = {}        # well -> {'X':int,'Y':int,'Z':int}
        self.named: dict[str, dict] = {}         # off-plate positions (WASTE, PARK, ...)
        self.z_travel_usteps = z_travel_usteps   # absolute travel Z (firmware usteps)
        # Marks the joint-count unit the data was taught in: None/8 = legacy
        # full-step firmware, 256 = v2 microstep firmware. goto refuses to replay
        # data taught at the wrong scale (it would be 32x off).
        self.ustep_scale = ustep_scale

    # -------------------------------------------------------------- teaching
    def teach(self, well_id, x, y, z):
        self.taught[well_id.strip().upper()] = {
            "X": int(round(x)), "Y": int(round(y)), "Z": int(round(z))}

    def forget(self, well_id):
        self.taught.pop(well_id.strip().upper(), None)

    def is_taught(self, well_id) -> bool:
        return well_id.strip().upper() in self.taught

    # --------------------------------------------- named (off-plate) positions
    def teach_named(self, name, x, y, z):
        """Save a NAMED off-plate position (e.g. WASTE, PARK) in joint usteps."""
        self.named[name.strip().upper()] = {
            "X": int(round(x)), "Y": int(round(y)), "Z": int(round(z))}

    def forget_named(self, name):
        self.named.pop(name.strip().upper(), None)

    def is_named(self, name) -> bool:
        return name.strip().upper() in self.named

    def joint_for_named(self, name) -> dict:
        return dict(self.named[name.strip().upper()])

    def corners_present(self, plate=None) -> bool:
        corners = plate_corners(plate) if plate is not None else CORNERS
        return all(c in self.taught for c in corners)

    # ----------------------------------------------------------- resolution
    def joint_for_well(self, well_id, plate) -> dict:
        """Return {'X','Y','Z'} joint usteps for a well (taught or interpolated)."""
        w = well_id.strip().upper()
        if w in self.taught:
            return dict(self.taught[w])
        corners = plate_corners(plate)        # real plate corners (96 or 384, etc.)
        if not self.corners_present(plate):
            missing = [c for c in corners if c not in self.taught]
            raise KeyError(
                f"well {w} is not taught and the corners {missing} are not all "
                f"taught yet (need {corners} to interpolate). Teach it directly.")
        row, col = plate.parse_well_id(w)
        n_rows = len(plate.rows)
        n_cols = len(plate.columns)
        u = col / (n_cols - 1) if n_cols > 1 else 0.0   # 0 at first col .. 1 at last
        v = row / (n_rows - 1) if n_rows > 1 else 0.0   # 0 at first row .. 1 at last
        a1, a12, h1, h12 = (self.taught[c] for c in corners)
        out = {}
        for axis in ("X", "Y", "Z"):
            out[axis] = int(round(
                (1 - u) * (1 - v) * a1[axis] + u * (1 - v) * a12[axis]
                + (1 - u) * v * h1[axis] + u * v * h12[axis]))
        return out

    # --------------------------------------------------------------- travel
    def travel_z(self, plate=None) -> int | None:
        """Absolute travel Z (usteps). If unset, use the highest taught Z."""
        if self.z_travel_usteps is not None:
            return self.z_travel_usteps
        zs = [v["Z"] for v in self.taught.values()]
        return max(zs) if zs else None

    # ---------------------------------------------------------------- persist
    def to_dict(self) -> dict:
        return {"labware": self.labware,
                "z_travel_usteps": self.z_travel_usteps,
                "ustep_scale": self.ustep_scale,
                "taught": self.taught,
                "named": self.named}

    def save(self, path=None) -> str:
        path = path or DEFAULT_TEACH_PATH
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2)
        return path

    @classmethod
    def load(cls, path=None):
        path = path or DEFAULT_TEACH_PATH
        if not os.path.isfile(path):
            return cls()
        with open(path) as f:
            d = json.load(f)
        t = cls(labware=d.get("labware", "eppendorf_twintec_lobind_96_pcr.json"),
                z_travel_usteps=d.get("z_travel_usteps"),
                ustep_scale=d.get("ustep_scale"))
        t.taught = {k.upper(): v for k, v in d.get("taught", {}).items()}
        t.named = {k.upper(): v for k, v in d.get("named", {}).items()}
        return t

    def summary(self) -> str:
        n = len(self.taught)
        named = (f"; named: {', '.join(sorted(self.named))}" if self.named else "")
        if n == 0:
            return ("teach table: EMPTY (jog to wells and `teach <well>`)" if not self.named
                    else f"teach table: 0 wells taught{named}")
        corners = "corners OK" if self.corners_present() else "corners INCOMPLETE"
        tz = self.travel_z()
        return (f"teach table: {n} well(s) taught [{', '.join(sorted(self.taught))}]; "
                f"{corners}; travel Z={'auto' if self.z_travel_usteps is None else tz}{named}")
