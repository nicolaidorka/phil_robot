"""TeachTable.predict_grid — learn untaught wells from taught wells + the rigid 9mm grid.

No hardware, no scipy: pure numpy lattice fit over (col,row) indices.
"""


def _plate():
    from phil.geometry.well_plate import WellPlate
    return WellPlate.load()


def test_grid_recovers_smooth_map():
    # A bilinear joint surface is exactly in the predictor's basis (1,c,r,c*r), so a
    # 2-D spread of taught wells must recover any held-out well to rounding.
    from phil.geometry.teach import TeachTable
    plate = _plate()

    def true_j(w):
        r, c = plate.parse_well_id(w)
        X = 600 + 1300 * c + 80 * r + 5 * c * r
        Y = 800 - 800 * r + 50 * c + 2 * c * r
        return int(X), int(Y)

    t = TeachTable()
    for w in ("A1", "A12", "H1", "H12", "D6", "E7", "C4", "F9", "B3", "G10"):
        X, Y = true_j(w)
        t.taught[w] = {"X": X, "Y": Y, "Z": 0}

    for held in ("D5", "F6", "C8", "G2", "B11"):
        p = t.predict_grid(held, plate)
        X, Y = true_j(held)
        assert p is not None, held
        assert abs(p["X"] - X) <= 2 and abs(p["Y"] - Y) <= 2, (held, p, (X, Y))


def test_grid_collinear_is_graceful():
    # All taught wells in ONE row -> row index never varies. The fit must NOT invent a
    # row slope (no r term), must not throw, and an off-row well predicts the col-only
    # value (same as any other row at that column).
    from phil.geometry.teach import TeachTable
    plate = _plate()
    t = TeachTable()
    for c in range(12):                       # row A, columns 1..12
        t.taught[f"A{c + 1}"] = {"X": 600 + 100 * c, "Y": 800 - 5 * c, "Z": 0}

    p = t.predict_grid("D6", plate)           # D6 -> (row 3, col 5); col-only formula
    assert p is not None
    assert p["X"] == 600 + 100 * 5 and p["Y"] == 800 - 5 * 5     # row ignored, no garbage slope
    # off-row prediction equals the same column regardless of row (no r dependence)
    assert (p["X"], p["Y"]) == (t.predict_grid("H6", plate)["X"],
                                t.predict_grid("H6", plate)["Y"])


def test_grid_too_few_wells_returns_none():
    # <3 taught -> None, so a single taught well is never predicted here (it exact-replays
    # via the taught branch in _resolve_raw instead).
    from phil.geometry.teach import TeachTable
    plate = _plate()
    t = TeachTable()
    t.taught["A1"] = {"X": 600, "Y": 800, "Z": 0}
    assert t.predict_grid("D6", plate) is None
    t.taught["A12"] = {"X": 14000, "Y": -9000, "Z": 0}
    assert t.predict_grid("D6", plate) is None


def test_grid_loo_flags_a_mis_taught_well():
    # Leave-one-out: a well moved far off the grid should be the worst LOO residual.
    from phil.geometry.teach import TeachTable
    plate = _plate()
    t = TeachTable()
    for w in ("A1", "A12", "H1", "H12", "D6", "E7", "C4", "F9", "B3", "G10"):
        r, c = plate.parse_well_id(w)
        t.taught[w] = {"X": 600 + 1300 * c + 80 * r, "Y": 800 - 800 * r + 50 * c, "Z": 0}
    t.taught["E7"]["X"] += 6000               # corrupt E7 by ~50mm-class error
    ranked = t.grid_loo(plate)
    assert ranked and ranked[0][0] == "E7", ranked
    assert ranked[0][1] > 1000                # large residual, usteps
