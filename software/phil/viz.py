"""Shared matplotlib helpers for Phil plate visualizations.

Kept backend-agnostic: this module imports only ``matplotlib.patches`` /
``matplotlib.collections`` (never ``pyplot``), so callers stay free to choose an
interactive backend (the live tracker) or Agg (the static diagnostics) before
they import pyplot themselves.
"""
from __future__ import annotations

from .geometry.teach import plate_corners

# axis labels every plate plot shares (everything is in the labware JSON frame)
XLABEL = "plate-local X (mm)"
YLABEL = "plate-local Y (mm)"


def plate_bounds(plate, margin: float = 12.0):
    """(xmin, xmax, ymin, ymax) of the well centres, padded by ``margin`` mm."""
    xs = [plate.local_xy(w)[0] for w in plate.well_ids()]
    ys = [plate.local_xy(w)[1] for w in plate.well_ids()]
    return (min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin)


def draw_plate_grid(ax, plate, *, edgecolor="#1f77b4", lw=1.0, alpha=0.8,
                    label_corners=True, margin=12.0, set_frame=True):
    """Draw every well as an open circle (true diameter) and frame the axes.

    Returns the matplotlib axes. Coordinates are the labware JSON frame, which is
    the same frame ``KinematicModel.forward`` returns -- so any tip/marker drawn
    in mm lands on the right cell with no transform.
    """
    from matplotlib.patches import Circle

    for w in plate.well_ids():
        x, y = plate.local_xy(w)
        ax.add_patch(Circle((x, y), plate.well(w).diameter / 2.0,
                            fill=False, ec=edgecolor, lw=lw, alpha=alpha))

    if label_corners:
        for c in plate_corners(plate):
            if c in plate:
                x, y = plate.local_xy(c)
                ax.annotate(c, (x, y), textcoords="offset points", xytext=(5, 5),
                            fontsize=8, color="#444444", weight="bold")

    if set_frame:
        x0, x1, y0, y1 = plate_bounds(plate, margin)
        ax.set_xlim(x0, x1)
        ax.set_ylim(y0, y1)
        ax.set_aspect("equal")
        ax.set_xlabel(XLABEL)
        ax.set_ylabel(YLABEL)
    return ax
