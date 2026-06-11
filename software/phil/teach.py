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

DEFAULT_TEACH_PATH = os.path.join(os.path.dirname(__file__), "config", "phil_teach.json")
CORNERS = ("A1", "A12", "H1", "H12")


class TeachTable:
    def __init__(self, labware="eppendorf_twintec_lobind_96_pcr.json",
                 z_travel_usteps=None):
        self.labware = labware
        self.taught: dict[str, dict] = {}        # well -> {'X':int,'Y':int,'Z':int}
        self.z_travel_usteps = z_travel_usteps   # absolute travel Z (repo usteps)

    # -------------------------------------------------------------- teaching
    def teach(self, well_id, x, y, z):
        self.taught[well_id.strip().upper()] = {
            "X": int(round(x)), "Y": int(round(y)), "Z": int(round(z))}

    def forget(self, well_id):
        self.taught.pop(well_id.strip().upper(), None)

    def is_taught(self, well_id) -> bool:
        return well_id.strip().upper() in self.taught

    def corners_present(self) -> bool:
        return all(c in self.taught for c in CORNERS)

    # ----------------------------------------------------------- resolution
    def joint_for_well(self, well_id, plate) -> dict:
        """Return {'X','Y','Z'} joint usteps for a well (taught or interpolated)."""
        w = well_id.strip().upper()
        if w in self.taught:
            return dict(self.taught[w])
        if not self.corners_present():
            missing = [c for c in CORNERS if c not in self.taught]
            raise KeyError(
                f"well {w} is not taught and the corners {missing} are not all "
                f"taught yet (need {CORNERS} to interpolate). Teach it directly.")
        row, col = plate.parse_well_id(w)
        n_rows = len(plate.rows)
        n_cols = len(plate.columns)
        u = col / (n_cols - 1) if n_cols > 1 else 0.0   # 0 at col1 .. 1 at col12
        v = row / (n_rows - 1) if n_rows > 1 else 0.0   # 0 at row A .. 1 at row H
        a1, a12, h1, h12 = (self.taught[c] for c in CORNERS)
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
                "taught": self.taught}

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
                z_travel_usteps=d.get("z_travel_usteps"))
        t.taught = {k.upper(): v for k, v in d.get("taught", {}).items()}
        return t

    def summary(self) -> str:
        n = len(self.taught)
        if n == 0:
            return "teach table: EMPTY (jog to wells and `teach <well>`)"
        corners = "corners OK" if self.corners_present() else "corners INCOMPLETE"
        tz = self.travel_z()
        return (f"teach table: {n} well(s) taught [{', '.join(sorted(self.taught))}]; "
                f"{corners}; travel Z={'auto' if self.z_travel_usteps is None else tz}")
