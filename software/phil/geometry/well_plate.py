"""Well-plate geometry for the Phil robot.

Loads an Opentrons-format labware JSON (see ``labware/``) and exposes the
*plate-local* coordinates of every well.  Plate-local coordinates are the
millimetre positions printed in the labware file; they say nothing about
where the plate physically sits on the robot.  Turning a plate-local
coordinate into a robot coordinate is the job of :mod:`phil.calibration`.

A well id is a row letter + a column number, e.g. ``"A1"`` .. ``"H12"``.
"""
from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass

_WELL_RE = re.compile(r"^([A-Za-z]+)([0-9]+)$")

# Labware search dirs (custom_labware first) and the default plate live in phil.paths.
# Search order: custom_labware (copied from ~/ms_sp) then bundled labware/.
# Default plate physically on Phil: Eppendorf twin.tec LoBind 96 PCR.
from ..paths import LABWARE_DIRS, DEFAULT_LABWARE


def resolve_labware(name_or_path: str) -> str:
    """Resolve a labware reference to a JSON path.

    Accepts a direct path, or a name/loadName/displayName (with or without the
    .json extension) found in the custom_labware / labware folders.
    """
    if os.path.isfile(name_or_path):
        return name_or_path
    cands = [name_or_path, name_or_path + ".json"]
    for d in LABWARE_DIRS:
        for c in cands:
            p = os.path.join(d, c)
            if os.path.isfile(p):
                return p
    # last resort: case-insensitive match on filename stem
    target = name_or_path.lower().removesuffix(".json")
    for d in LABWARE_DIRS:
        if not os.path.isdir(d):
            continue
        for fn in os.listdir(d):
            if fn.lower().removesuffix(".json") == target:
                return os.path.join(d, fn)
    raise FileNotFoundError(
        f"labware {name_or_path!r} not found in {LABWARE_DIRS}. "
        f"Available: {available_labware()}")


def available_labware() -> list[str]:
    """List labware definition filenames available to load by name."""
    out = []
    for d in LABWARE_DIRS:
        if os.path.isdir(d):
            out += [fn for fn in sorted(os.listdir(d)) if fn.endswith(".json")]
    return out


@dataclass(frozen=True)
class Well:
    """A single well's plate-local geometry (millimetres)."""

    id: str
    row: int          # 0-based (A=0)
    col: int          # 0-based (column 1 -> 0)
    x: float          # plate-local center x
    y: float          # plate-local center y
    z_bottom: float   # plate-local z of the well bottom
    depth: float      # well cavity depth
    diameter: float   # well opening diameter


class WellPlate:
    """Parsed labware definition with plate-local well coordinates."""

    def __init__(self, definition: dict):
        self._def = definition
        self.load_name = definition.get("parameters", {}).get("loadName", "unknown")
        self.display_name = definition.get("metadata", {}).get("displayName", self.load_name)
        self.dimensions = definition.get("dimensions", {})

        grid = definition.get("philGrid", {})
        self.rows = grid.get("rows") or self._infer_rows(definition)
        self.columns = grid.get("columns") or self._infer_columns(definition)
        self.row_spacing_mm = grid.get("rowSpacingMM")
        self.column_spacing_mm = grid.get("columnSpacingMM")

        self._wells: dict[str, Well] = {}
        for well_id, w in definition["wells"].items():
            row, col = self.parse_well_id(well_id)
            self._wells[well_id.upper()] = Well(
                id=well_id.upper(),
                row=row,
                col=col,
                x=float(w["x"]),
                y=float(w["y"]),
                z_bottom=float(w.get("z", 0.0)),
                depth=float(w.get("depth", 0.0)),
                diameter=float(w.get("diameter", 0.0)),
            )

    # -- construction ---------------------------------------------------------
    @classmethod
    def load(cls, path: str | None = None) -> "WellPlate":
        path = DEFAULT_LABWARE if path is None else resolve_labware(path)
        with open(path, "r") as f:
            return cls(json.load(f))

    # -- well id parsing ------------------------------------------------------
    @staticmethod
    def parse_well_id(well_id: str) -> tuple[int, int]:
        """``"B3"`` -> ``(row=1, col=2)`` (both 0-based)."""
        m = _WELL_RE.match(well_id.strip())
        if not m:
            raise ValueError(f"invalid well id: {well_id!r} (expected e.g. 'A1', 'H12')")
        row_letters, col_digits = m.groups()
        row_letters = row_letters.upper()
        row = 0
        for ch in row_letters:                      # supports A..Z, AA.. for big plates
            row = row * 26 + (ord(ch) - ord("A") + 1)
        row -= 1
        col = int(col_digits) - 1
        if row < 0 or col < 0:
            raise ValueError(f"invalid well id: {well_id!r}")
        return row, col

    # -- access ---------------------------------------------------------------
    def well(self, well_id: str) -> Well:
        key = well_id.strip().upper()
        if key not in self._wells:
            raise KeyError(f"well {well_id!r} not in labware {self.load_name!r}")
        return self._wells[key]

    def local_xy(self, well_id: str) -> tuple[float, float]:
        w = self.well(well_id)
        return w.x, w.y

    def well_ids(self) -> list[str]:
        return list(self._wells.keys())

    def __contains__(self, well_id: str) -> bool:
        try:
            return well_id.strip().upper() in self._wells
        except AttributeError:
            return False

    def __len__(self) -> int:
        return len(self._wells)

    def __repr__(self) -> str:
        return f"<WellPlate {self.load_name!r} {len(self)} wells>"

    # -- fallbacks if the labware lacks a philGrid block ----------------------
    @staticmethod
    def _infer_rows(definition: dict) -> list[str]:
        rows = sorted({WellPlate.parse_well_id(w)[0] for w in definition["wells"]})
        return [chr(ord("A") + r) for r in rows]

    @staticmethod
    def _infer_columns(definition: dict) -> list[int]:
        cols = sorted({WellPlate.parse_well_id(w)[1] for w in definition["wells"]})
        return [c + 1 for c in cols]
