"""Re-referencing an insert to the part's measured frame, checked without a cell.

The shell comes to rest about 4 degrees out of square on its narrow rails, and over the
pin's 138 mm stroke 4 degrees is 10 mm at the far end against a 1.5 mm chamfer. So the
insert is transformed into the frame the shell is really in rather than refused.

A transposed rotation is the way that goes wrong silently: it still produces a pose, the
arm still goes there, and the part is inserted at the MIRROR of the correction it needed.
These tests assert the properties a transpose does not preserve.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "probes"))

from assembly.asset import insertions                            # noqa: E402
from assembly.build_fragment import chosen                        # noqa: E402
from assembly.asset import part                                   # noqa: E402
from assembly.film_cycle import into_frame                        # noqa: E402
from assembly.stations import TOOLS, jaw_extra_mm, stations_for    # noqa: E402

# The pin's grip centre from the flange, as the asset publishes it (ezuTcpOffset).
JAW_PAD_MM = TOOLS["jaw"].standoff_mm + jaw_extra_mm("Pin")

CELL = chosen()
# The pin, because it has by far the longest stroke and so the most to lose to a tilt.
# Read from the cell rather than copied: the belt stop and the stand-off have both moved.
_PIN_ST = next(s for s in stations_for(CELL) if s.task == "pin_insert")
PIN_INDEX_MM = CELL.index_x("pin")
PIN_FLANGE_MM = list(_PIN_ST.flange_mm())
PIN_APPROACH = [0.0, 1.0, 0.0]


def _rot(axis: int, deg: float):
    """The three axis vectors of a frame rotated `deg` about one world axis."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    full = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    i, j = [(1, 2), (2, 0), (0, 1)][axis]
    full[i][i], full[i][j] = c, s
    full[j][i], full[j][j] = -s, c
    return full


def _nominal():
    return (PIN_INDEX_MM, CELL.belt_y_mm, CELL.shell_origin_mm[2])


def test_a_square_shell_is_not_moved_at_all():
    # The transform has to be the identity when there is nothing to correct, or every
    # cycle pays for a correction it does not need - and the measurement of whether a
    # correction is wanted is exactly this distance.
    nom = _nominal()
    at = into_frame([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], list(nom),
                    [PIN_FLANGE_MM[i] - nom[i] for i in range(3)], point=True)
    assert at == pytest.approx(PIN_FLANGE_MM)


@pytest.mark.parametrize("axis,deg", [(0, 4.2), (0, -4.2), (1, 3.0), (2, 5.0),
                                      (0, 9.0), (2, -9.0)])
def test_the_stroke_still_lands_on_the_bore_however_the_shell_sits(axis, deg):
    """The property: the pin ends exactly as far from its seated position as when square.

    If the whole insert is carried into the shell's frame, the geometry between the tool
    and the bore is RIGID - so the stroke's end point sits the same distance from the
    pin's seated position as the taught stroke did from the nominal one. A transposed
    rotation, a rotation applied to the direction but not the point, or a pivot taken
    about the wrong origin all break that.
    """
    ins = insertions()["Pin"]
    nom = _nominal()
    rot = _rot(axis, deg)
    # Put the shell somewhere non-trivial as well as rotated, so a missing translation
    # cannot pass.
    live = [nom[0] + 3.0, nom[1] - 2.0, nom[2] + 1.5]

    at_f = into_frame(rot, live, [PIN_FLANGE_MM[i] - nom[i] for i in range(3)], point=True)
    at_axis = into_frame(rot, live, ins.axis, point=False)
    end = [at_f[i] + at_axis[i] * ins.depth_mm for i in range(3)]
    seated = into_frame(rot, live, ins.seated_mm, point=True)

    # What the taught, square insert achieved.
    square_end = [PIN_FLANGE_MM[i] + ins.axis[i] * ins.depth_mm for i in range(3)]
    square_seated = [nom[i] + ins.seated_mm[i] for i in range(3)]
    assert math.dist(end, seated) == pytest.approx(
        math.dist(square_end, square_seated), abs=1e-6)
    # And at the end of the stroke the jaw's pads are centred on the seated pin's grasp
    # point - the check that the taught pose and the published seat agreed in the first
    # place.
    grasp = [v * 1000.0 for v in part("Pin").at("assembly:graspPosition")]
    square_grasp = [square_seated[i] + grasp[i] for i in range(3)]
    assert math.dist(square_end, square_grasp) == pytest.approx(JAW_PAD_MM, abs=0.1)


@pytest.mark.parametrize("axis,deg", [(0, 4.2), (2, 5.0)])
def test_the_stroke_runs_along_the_tilted_bore_not_the_world_axis(axis, deg):
    """The correction's whole point: the pin stays COAXIAL with the bore.

    A lateral-only correction aims at the mouth of the hole and enters at an angle, which
    in a 0.20 mm bore levers rather than inserts. The stroke direction therefore has to
    carry the shell's rotation, and it has to be the SAME rotation the pose carried - the
    angle between the two is the thing that tells them apart.
    """
    ins = insertions()["Pin"]
    rot = _rot(axis, deg)
    at_axis = into_frame(rot, [0.0, 0.0, 0.0], ins.axis, point=False)
    # It has moved off the world axis by exactly the shell's tilt...
    dot = sum(at_axis[i] * ins.axis[i] for i in range(3))
    assert math.degrees(math.acos(max(-1.0, min(1.0, dot)))) == pytest.approx(
        abs(deg), abs=1e-6)
    # ...and it is still a unit vector, which a sloppy rotation is not.
    assert math.sqrt(sum(v * v for v in at_axis)) == pytest.approx(1.0, abs=1e-9)
    # And the tool's approach moves with it: the flange still stands off ALONG the bore,
    # so the tool axis and the stroke stay anti-parallel.
    at_app = into_frame(rot, [0.0, 0.0, 0.0], PIN_APPROACH, point=False)
    assert sum(at_app[i] * at_axis[i] for i in range(3)) == pytest.approx(-1.0, abs=1e-9)


def test_a_transposed_rotation_would_be_caught():
    """The guard on the guard: show the property actually discriminates.

    A test that passes for both a rotation and its transpose is a test that is not
    checking the thing it names. With the shell rotated about two axes at once the two
    differ, and this asserts that they do - so the property above has teeth.
    """
    ins = insertions()["Pin"]
    nom = _nominal()
    a = _rot(0, 4.2)
    b = _rot(2, 5.0)
    rot = [[sum(a[k][i] * b[j][k] for k in range(3)) for i in range(3)] for j in range(3)]
    transposed = [[rot[i][j] for i in range(3)] for j in range(3)]
    local = [PIN_FLANGE_MM[i] - nom[i] for i in range(3)]

    def end_gap(r):
        at_f = into_frame(r, list(nom), local, point=True)
        at_axis = into_frame(r, list(nom), ins.axis, point=False)
        end = [at_f[i] + at_axis[i] * ins.depth_mm for i in range(3)]
        return math.dist(end, into_frame(r, list(nom), ins.seated_mm, point=True))

    # The true rotation preserves the standoff; its transpose is a different frame, so the
    # POSE it produces is somewhere else entirely - which is what the live `moved` reading
    # would show as a correction in the wrong direction.
    # (Measured to the seated pin's ORIGIN: the pads' 162.5 mm plus the grasp point's
    # 26 mm along the axis from it.)
    grasp_y = part("Pin").at("assembly:graspPosition")[1] * 1000.0
    assert end_gap(rot) == pytest.approx(JAW_PAD_MM + grasp_y, abs=0.1)
    true_f = into_frame(rot, list(nom), local, point=True)
    tran_f = into_frame(transposed, list(nom), local, point=True)
    assert math.dist(true_f, tran_f) > 5.0


# ---- turning the tool onto the bore's axis ------------------------------------------

def test_turn_onto_lands_the_axis_exactly():
    """The correction for a part held at an angle, which is worth 4.8 mm per 2 degrees.

    Measured on the cell: the pin finished "0.04 mm short of full depth and 4.76 mm across
    the bore" - a stroke that went exactly as far as it was told and arrived sideways,
    because the align puts the part's ORIGIN on the line and says nothing about which way
    it points. 138 x sin(2 degrees) is 4.82 mm.
    """
    from assembly.film_cycle import turn_onto

    for axis, deg in ((0, 2.0), (1, 5.0), (2, -3.0), (0, 0.4)):
        rot = _rot(axis, deg)
        # A part whose own axis has been turned off +Y by that much.
        have = into_frame(rot, [0.0, 0.0, 0.0], [0.0, 1.0, 0.0], point=False)
        want = [0.0, 1.0, 0.0]
        got = into_frame(turn_onto(have, want), [0.0, 0.0, 0.0], have, point=False)
        assert got == pytest.approx(want, abs=1e-9)


def test_turn_onto_is_the_identity_when_there_is_nothing_to_turn():
    # A correction that fires when the part is already square costs a planned move for
    # nothing, so the floor above it has to be reachable.
    from assembly.film_cycle import turn_onto

    v = [0.0, 1.0, 0.0]
    assert into_frame(turn_onto(v, v), [0.0, 0.0, 0.0], [1.0, 2.0, 3.0],
                      point=False) == pytest.approx([1.0, 2.0, 3.0], abs=1e-12)


def test_turn_onto_is_a_rotation_not_a_squash():
    """Lengths and angles survive it, which a hand-built matrix does not always manage."""
    from assembly.film_cycle import turn_onto

    have = into_frame(_rot(2, 7.0), [0.0, 0.0, 0.0], [0.0, 1.0, 0.0], point=False)
    rot = turn_onto(have, [0.0, 1.0, 0.0])
    for v in ([1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 2.0, -3.0]):
        out = into_frame(rot, [0.0, 0.0, 0.0], v, point=False)
        assert math.sqrt(sum(x * x for x in out)) == pytest.approx(
            math.sqrt(sum(x * x for x in v)), abs=1e-9)
    # ...and it does not mirror: the determinant is +1, not -1.
    cols = [[rot[j][i] for j in range(3)] for i in range(3)]
    det = (cols[0][0] * (cols[1][1] * cols[2][2] - cols[1][2] * cols[2][1])
           - cols[0][1] * (cols[1][0] * cols[2][2] - cols[1][2] * cols[2][0])
           + cols[0][2] * (cols[1][0] * cols[2][1] - cols[1][1] * cols[2][0]))
    assert det == pytest.approx(1.0, abs=1e-9)


# ---- the pick and the seat have to agree about where the tool holds the part ---------

@pytest.mark.parametrize("part_name,pick,seat", [("Pin", "pin_pick", "pin_insert"),
                                                 ("Flag", "flag_pick", "flag_insert")])
def test_the_seat_holds_the_part_the_same_way_the_pick_took_it(part_name, pick, seat):
    """The invariant that retires a whole class of correction.

    A station places the FLANGE at `work + approach * (standoff + extra)`, a scalar along
    one axis. The pick leaves the flange a known distance from the part's GRASP POINT, so
    the seat can only reproduce that relationship if its work point is the grasp point
    too. Aiming at the part's ORIGIN instead left the two differing by the grasp offset -
    205 mm on the flag, whose grasp shaft is at the far end of a 252 mm blade - and the
    live correction had to step exactly that, 208.19 mm, with the blade swinging on the
    end of it.

    `stations.py` opens by naming "a derived scalar true of one grasp reused for another"
    as the bug class the module is arranged against. This is that invariant, asserted.
    """
    from assembly.stations import stations_for as _s

    st = {s.task: s for s in _s(CELL)}
    p, q = st[pick], st[seat]
    took = [p.flange_mm()[i] - p.work_mm[i] for i in range(3)]
    holds = [q.flange_mm()[i] - q.work_mm[i] for i in range(3)]
    # In the PART's frame: the feeder may hold the part turned (the pin stands up in its
    # nest), so the pick's world offset is rotated back by the feeder's pose before it is
    # compared with the seat's, where the part sits square to the shell.
    from assembly.fixture import _quat_rotate, feed_orientation
    w, x, y, z = feed_orientation(part_name)
    took = list(_quat_rotate((w, -x, -y, -z), took))
    if part_name == "Flag":
        # Except by what the grip is MEASURED to hold the flag deeper than the jaw model
        # (stations.FLAG_HELD_DEEPER_MM): the seat models the hold the cell really has.
        from assembly.stations import FLAG_HELD_DEEPER_MM
        n = math.sqrt(sum(v * v for v in took))
        took = [v * (n + FLAG_HELD_DEEPER_MM) / n for v in took]
    assert holds == pytest.approx(took, abs=0.01), (
        f"{part_name}: the pick leaves the flange {took} from the grasp point and the "
        f"seat expects it {holds} - the difference is a correction every cycle has to "
        f"make, with the part swinging on the end of it")
    # And the work point really is the grasp point, not the origin - which is the thing
    # that makes the equality above true rather than a coincidence of two wrong numbers.
    grasp = [v * 1000.0 for v in part(part_name).at("assembly:graspPosition")]
    assert math.sqrt(sum(g * g for g in grasp)) > 1.0, (
        f"{part_name} grasps at its own origin, so this test proves nothing - pick a part "
        f"whose grasp is offset, or drop the test")


# ---- the part has to actually line up with the feature it goes into -----------------

def test_the_flags_peg_lands_on_its_hole_when_seated():
    """Checked by hand while chasing a 3.60 mm miss; kept so nobody has to check it again.

    If a part's published seated position does not put its own peg on the hole's axis,
    every correction downstream is aiming at a target that was never right, and the miss
    looks exactly like a dynamic problem. It is worth one test to take that off the table
    permanently.
    """
    from assembly.colliders import pieces

    shell = part("Shell")
    ins = insertions()["Flag"]
    hole = shell.mm_at("assembly:flagHole:entry")
    peg = next(p for p in pieces("Flag") if p["name"] == "Peg")
    seated = ins.seated_mm

    # Across the hole's axis (which is -Y), the peg's centre sits on the hole's centre.
    for i in (0, 2):
        centre = seated[i] + (peg["min_mm"][i] + peg["max_mm"][i]) / 2.0
        assert centre == pytest.approx(hole[i], abs=0.05), (
            f"the flag's peg centre is {centre:.2f} on axis {i} and its hole is "
            f"{hole[i]:.2f} - the seat is aiming where the peg is not")
    # Along the axis, the peg's mouth meets the hole's mouth.
    assert seated[1] + peg["max_mm"][1] == pytest.approx(hole[1], abs=0.05)
    # And it really is a clearance fit, not an interference one: the published radial
    # slack is the half-difference of the two diameters.
    wide = peg["max_mm"][0] - peg["min_mm"][0]
    assert (shell.mm_at("assembly:flagHole:diameter") - wide) / 2.0 == pytest.approx(
        ins.clearance_mm, abs=0.05)
