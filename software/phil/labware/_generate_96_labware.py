"""Generate the Opentrons-format 96-well plate labware definition.

This emits ``corning_96_wellplate_360ul_flat.json`` with accurate SBS
footprint geometry.  Re-run only if you need to regenerate the file:

    python software/phil/labware/_generate_96_labware.py

All measurements are in millimetres, following the Opentrons labware
schema (schemaVersion 2): the plate origin is the front-left-bottom
corner, +x runs left->right (columns 1..12), +y runs front->back, and
well coordinates give the *center* of the well at the *top* of the well
cavity is `z = zDimension`; the `z` value stored per well is the well
bottom measured from the plate bottom.
"""
import json
import os

# --- SBS / Corning 3596-style 96-well flat-bottom plate, in mm ---------------
PLATE = {
    "x_dimension": 127.76,   # plate length (SBS footprint)
    "y_dimension": 85.48,    # plate width  (SBS footprint)
    "z_dimension": 14.22,    # plate height (top of plate from deck)
}
WELL = {
    "depth": 10.67,          # depth of the well cavity
    "diameter": 6.86,        # circular well opening diameter
    "total_liquid_volume": 360.0,
    "shape": "circular",
}
SPACING_MM = 9.0             # center-to-center, both axes (SBS standard)
A1_X_MM = 14.38              # center of A1 from left edge
A1_Y_MM = 74.24              # center of A1 from front edge (back-left well)
WELL_BOTTOM_Z_MM = round(PLATE["z_dimension"] - WELL["depth"], 2)  # 3.55

ROWS = "ABCDEFGH"            # 8 rows
N_COLS = 12                  # 12 columns


def build():
    wells = {}
    ordering = []
    for col in range(N_COLS):              # 0..11  -> columns 1..12
        column_ids = []
        for row in range(len(ROWS)):       # 0..7   -> rows A..H
            well_id = f"{ROWS[row]}{col + 1}"
            wells[well_id] = {
                "depth": WELL["depth"],
                "totalLiquidVolume": WELL["total_liquid_volume"],
                "shape": WELL["shape"],
                "diameter": WELL["diameter"],
                # plate-local coordinates of the well center:
                "x": round(A1_X_MM + col * SPACING_MM, 2),
                "y": round(A1_Y_MM - row * SPACING_MM, 2),
                "z": WELL_BOTTOM_Z_MM,
            }
            column_ids.append(well_id)
        ordering.append(column_ids)

    return {
        "ordering": ordering,
        "brand": {"brand": "Corning", "brandId": ["3596"]},
        "metadata": {
            "displayName": "Corning 96 Well Plate 360 uL Flat",
            "displayCategory": "wellPlate",
            "displayVolumeUnits": "uL",
            "tags": [],
        },
        "dimensions": {
            "xDimension": PLATE["x_dimension"],
            "yDimension": PLATE["y_dimension"],
            "zDimension": PLATE["z_dimension"],
        },
        "wells": wells,
        "groups": [
            {
                "metadata": {"wellBottomShape": "flat"},
                "wells": list(wells.keys()),
            }
        ],
        "parameters": {
            "format": "irregular",
            "isTiprack": False,
            "isMagneticModuleCompatible": False,
            "loadName": "corning_96_wellplate_360ul_flat",
        },
        "namespace": "phil",
        "version": 1,
        "schemaVersion": 2,
        "cornerOffsetFromSlot": {"x": 0, "y": 0, "z": 0},
        # Convenience block (non-standard, read by phil.well_plate) so the
        # robot layer can grab grid parameters without re-deriving them.
        "philGrid": {
            "rows": list(ROWS),
            "columns": list(range(1, N_COLS + 1)),
            "rowSpacingMM": SPACING_MM,
            "columnSpacingMM": SPACING_MM,
            "a1": {"x": A1_X_MM, "y": A1_Y_MM, "z": WELL_BOTTOM_Z_MM},
        },
    }


if __name__ == "__main__":
    out = os.path.join(os.path.dirname(__file__), "corning_96_wellplate_360ul_flat.json")
    with open(out, "w") as f:
        json.dump(build(), f, indent=2)
    print(f"wrote {out}")
