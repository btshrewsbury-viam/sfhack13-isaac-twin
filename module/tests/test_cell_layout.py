"""The cell's geometry, checked before any of it reaches a robot.

These are the properties the cell's design rests on. They were previously implied by a
stored pose file that nothing re-derived, and by the time anyone looked it described a
different grip, a different part and a cell with none of the current scenery.

Pure arithmetic over `probes/qc/layout.py`, so it runs without Isaac and fails the moment
someone moves a tray.
"""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "probes"))

from qc.layout import REACH_MM, Cell, reach_budget  # noqa: E402

# The side every grip in this cell uses, asked of the cell rather than restated here.
#
# This was `+y` - an END of the part - on the reasoning that the belt blocked the other
# side at the pick. Both halves of that are now wrong. The belt is a real conveyor and
# measurement picked the side: `-x` offers 86 mm of standoff, 480 mm of clear wall and
# nothing proud, where the ends need 229-252 mm and cantilever the full 495 mm off one
# cup - which is the grip the cell was explicitly asked not to make. `+x` is out too: it
# carries the flag AND both paint defects, and neither arm has IK for it.
PICK_SIDE = Cell.GRIP_SIDE
# The ends, which no arm grips. Several tests below reason about them anyway, to pin down
# WHY the layout is the way it is.
HELD_SIDE = "+y"
UNUSED_SIDE = "-y"


@pytest.fixture(scope="module")
def cell():
    return Cell.from_fragment()


def test_the_grip_side_is_the_one_with_wall_to_seal_on(cell):
    """The grip side is chosen by what the PART offers, not by what the scenery allows.

    This used to assert that only `+y` cleared the belt, on the reasoning that a slab
    running away in -y left no room to stand off on the other side. The belt is a real
    conveyor the part sits ON now, and the side was settled by measuring the part: `-x`
    gives 86 mm of standoff and 480 mm of clear wall with nothing proud of it, where the
    ends need 229-252 mm and would cantilever the full 495 mm off one cup.
    """
    faces = cell.grip_faces or {}
    chosen = faces[cell.GRIP_SIDE]
    assert not chosen["blocked_mm"], "the chosen side has something proud of it"
    for side, face in faces.items():
        if side == cell.GRIP_SIDE:
            continue
        assert chosen["clear_mm"] > face["clear_mm"], (
            f"{side} offers more clear wall ({face['clear_mm']:.0f} mm) than the side "
            f"the cell grips ({chosen['clear_mm']:.0f} mm)")


def test_the_pick_is_within_reach(cell):
    position, _ = cell.pick(PICK_SIDE)
    assert cell.reach_mm("arm-a", position) < reach_budget()


def test_the_handoff_is_mandatory_because_of_the_cup_not_the_reach(cell):
    """Why the cell has two arms.

    This used to assert that arm-a could not REACH either outfeed, which was true of
    the old layout and is no longer: putting the conveyors where arm-b can place on
    them comfortably - flange behind the part, tool pointing the way the arm reaches -
    brings the nearer one inside arm-a's envelope. That property was traded knowingly.

    What makes the handoff mandatory is stronger and was always the real reason: arm-a's
    cup is stuck to one face for the whole time it holds the part, so that face cannot
    be inspected by any pose arm-a can adopt. The part has to change hands or the cell
    cannot see all six faces. `test_round_e2e` asserts exactly that, on the simulator,
    and `test_face_derivation` shows the assertion can tell a turned handoff from an
    unturned one.
    """
    # Which face the cup is on is derived, not stated: each arm now stands off on its
    # OWN side of the part, so arm-a covers -y and arm-b +y. Hard-coded as "+y" this
    # passed for arm-a by luck and failed the moment the side changed.
    covered = cell.present_side("arm-a")
    for _, (pose) in enumerate(cell.present_poses("arm-a")):
        face, _ = pose
        assert face != covered, (
            "arm-a claims a presentation of the face its own cup is on")


def test_the_handoff_is_forced_whichever_side_is_held(cell):
    """arm-a cannot deliver to the outfeed, so the handoff is not decoration.

    What is asserted is the invariant: holding the side the belt leaves it - the only one
    it can take the part by - arm-a cannot reach the good conveyor, so the part has to
    change hands.

    WHY it cannot has changed, and that is worth recording rather than hiding. With the
    cube part this test asserted the opposite about the other side: holding the far side
    arm-a COULD have reached the tray, which made the belt the sole reason for the
    handoff. The mailbox is thinner than the cube - 63 mm against 110 - so the cup sits
    54 mm closer to the part's middle, and holding the far side now pushes the flange 54
    mm further across the cell, to 1020 mm against a measured limit of 1017. So the reach
    forces the handoff too, narrowly, and the design no longer rests on the belt alone.

    The far side's margin is reported, not asserted, because it is a fact about this part
    rather than a requirement on the cell.
    """
    held, _ = cell.place("good", HELD_SIDE)
    reach_held = cell.reach_mm("arm-a", held)
    assert reach_held >= REACH_MM, (
        f"arm-a can reach the good conveyor holding the {HELD_SIDE} side "
        f"({reach_held:.0f} mm against {REACH_MM:.0f}), so nothing forces the handoff "
        f"and the second check could be skipped entirely")
    far, _ = cell.place("good", UNUSED_SIDE)
    print(f"    holding {HELD_SIDE}: {reach_held:.0f} mm, "
          f"holding {UNUSED_SIDE}: {cell.reach_mm('arm-a', far):.0f} mm, "
          f"limit {REACH_MM:.0f}")


def test_arm_b_can_place_on_both_conveyors(cell):
    """Checked on the pose the round really uses.

    Two earlier versions checked poses no arm ever visits: first the -y flange, then
    the +y one, while the round lays the part down turned a quarter turn and puts its
    flange behind the part instead. Each time the test passed and the machine did not.
    """
    for tray in ("good", "bad"):
        position, _ = place_flange(cell, tray)
        assert cell.reach_mm("arm-b", position) < reach_budget(), (
            f"arm-b cannot place on the {tray} conveyor, so nothing can"
        )


def test_the_part_fits_the_inspection_frame(cell):
    """A presentation the camera cannot frame is not an inspection.

    fov and working distance were chosen for an 80 mm cube; the mailbox is 175 mm on its
    longest edge, and the frame at this distance was 159 mm.
    """
    import math

    distance = math.dist(cell.camera_mm, cell.camera_target_mm)
    config = __import__("json").loads(
        (Path(__file__).resolve().parents[1] / "fragments" / "qc-cell.json").read_text())
    camera = next(c for c in config["components"] if c["name"] == "inspect-cam")
    fov = float(camera["attributes"]["fov_deg"])
    frame_mm = 2.0 * distance * math.tan(math.radians(fov) / 2.0)
    longest = max(cell.part.dims_mm)
    assert frame_mm > longest * 1.25, (
        f"frame is {frame_mm:.0f} mm and the part is {longest:.0f} mm on its longest "
        f"edge; presentations overflow the image"
    )


def test_the_part_fits_inside_the_box_the_planner_is_told_about():
    """Every piece of the part that can COLLIDE must sit inside its declared box.

    The planner reasons about one box. A child reaching past it - the flag did, by
    30 mm - means paths get cleared that the real part does not fit through, and the
    failure looks like the arm stalling against nothing.

    Decals are exempt, and only decals. A defect mark carries no collider, so nothing
    can ever touch it and the planner's box is not lying by leaving it out; laying it
    flush with the surface instead would only buy a z-fight. What keeps that exemption
    from becoming a hole is `test_defects_carry_no_collider`, which is what makes a
    child a decal, and `test_defects_lie_on_the_part_rather_than_inside_it`, which
    bounds how far proud one may sit - a mark that stood off far enough to matter would
    be a dent or a proud door, and those are shapes, not paint.
    """
    import json

    config = json.loads(
        (Path(__file__).resolve().parents[1] / "fragments" / "qc-cell.json").read_text())
    world = next(c for c in config["components"] if c["model"].endswith(":world"))
    part = next(p for p in world["attributes"]["props"] if p["name"] == "part")
    if str(part.get("type", "cube")) == "usd":
        # An asset states the box directly, measured from its geometry, so there are no
        # children to reconcile with it - but the box must actually contain the part.
        bounds = part["bounds_m"]
        dims = Cell.from_fragment().part.dims_mm
        for axis in range(3):
            span = (bounds["max"][axis] - bounds["min"][axis]) * 1000.0
            assert span + 1e-6 >= dims[axis], (
                f"the declared box is {span:.1f} mm on {'xyz'[axis]} and the part "
                f"measures {dims[axis]:.1f}")
        return
    half = [v / 2 for v in part["scale"]]
    for child in part["children"]:
        if child.get("collision", True) is False:
            continue
        position = child.get("position", [0, 0, 0])
        if child.get("shape", "cube") == "cube":
            extent = [v / 2 for v in child["scale"]]
        else:
            radius = child.get("radius", 0.05)
            height = child.get("height", 0.1)
            extent = [height / 2 if child.get("axis") == "X" else radius,
                      radius, radius]
        for axis in range(3):
            reach = abs(position[axis]) + extent[axis]
            assert reach <= half[axis] + 1e-6, (
                f"{child['name']} reaches {reach * 1000:.0f} mm on axis {axis}, "
                f"past the declared {half[axis] * 1000:.0f} mm"
            )


def test_both_presentation_points_are_in_frame(cell):
    """Each arm presents on its own side of the camera axis, and both must be visible.

    The offset is not a style choice: an arm holding a flat side stands off 175 mm
    beyond the part, so pushing the part toward the arm is the only way to put its
    flange somewhere it can actually work. That pushes the part off the camera's axis,
    and the lens has to cover it.
    """
    import json
    import math

    config = json.loads(
        (Path(__file__).resolve().parents[1] / "fragments" / "qc-cell.json").read_text())
    camera = next(c for c in config["components"] if c["name"] == "inspect-cam")
    distance = math.dist(cell.camera_mm, cell.camera_target_mm)
    half_frame = distance * math.tan(
        math.radians(float(camera["attributes"]["fov_deg"])) / 2.0)
    for side in ("arm-a", "arm-b"):
        point = cell.inspect_point_mm(side)
        offset = abs(point[1] - cell.camera_target_mm[1])
        # Half the space diagonal, not half the longest side: a presentation rolls the
        # part to show each face, so the silhouette the lens has to cover is whatever
        # the part's widest orientation gives, and that bounds every one of them.
        reach = offset + math.dist((0, 0, 0), cell.part.dims_mm) / 2.0
        assert reach < half_frame, (
            f"presenting at y={point[1]:.0f} puts the part's edge {reach:.0f} mm off "
            f"axis, outside the {half_frame:.0f} mm half-frame"
        )


def test_the_camera_sits_between_the_two_presentation_points():
    """Both checks should frame the part the same way.

    The two arms present on opposite sides of the cell, so a camera on the centreline
    puts one check's part 185 mm off axis and the other's 15 mm - two quite different
    views of the same part, fed to one model. Aiming at the midpoint makes the worst
    case symmetric and halves it.
    """
    cell = Cell.from_fragment()
    points = [cell.inspect_point_mm(a)[1] for a in ("arm-a", "arm-b")]
    midpoint = sum(points) / 2.0
    assert abs(cell.camera_target_mm[1] - midpoint) < 1.0, (
        f"the camera aims at y={cell.camera_target_mm[1]:.0f} while the presentations "
        f"happen at {points[0]:.0f} and {points[1]:.0f}")
    assert abs(cell.camera_mm[1] - midpoint) < 1.0, (
        "the camera is not square on to the presentations")


def test_each_arm_handles_the_handoff_on_its_own_side(cell):
    """The set-down is turned half a turn, so neither arm reaches across.

    Unturned, arm-a's cup stays on the +y face and arm-b can only take the part by
    reaching past the centreline to its far side - which the arm reaches and then
    cannot finish, stalling the wrist 160 degrees short. Turned, arm-a's covered face
    lands on arm-a's side and the free face on arm-b's.
    """
    giver, _ = cell.fixture_grip("-y")
    taker, _ = cell.fixture_grip("+y")
    assert giver[1] < 0, "arm-a should set down on its own side of the cell"
    assert taker[1] > 0, "arm-b should pick up on its own side"
    assert cell.reach_mm("arm-a", giver) < reach_budget()
    assert cell.reach_mm("arm-b", taker) < reach_budget()


def test_the_two_arms_end_up_holding_opposite_faces_of_the_part():
    """Otherwise the second check re-inspects what the first one already saw.

    Both arms approach from world +y - the belt forces it at the pick, and the layout
    keeps arm-b on that side at the stand - so each cup lands on whichever face of the
    part happens to point along world +y at the time. What makes those different faces
    is the half turn between, so the test has to go through the turn.

    Two earlier versions of this compared the two flange positions either side of the
    stand, which is true by construction of `side_grip` for any cell at all. They passed
    whether or not the handoff turned anything, which is exactly the failure they
    existed to catch.
    """
    from qc.round import _quat_conj, _rotate_by_quat, yawed

    upright = (1.0, 0.0, 0.0, 0.0)

    def face_the_cup_lands_on(part_wxyz):
        body = _rotate_by_quat(_quat_conj(tuple(part_wxyz)), (0.0, 1.0, 0.0))
        axis = max(range(3), key=lambda i: abs(body[i]))
        return f"{'+' if body[axis] > 0 else '-'}{'xyz'[axis]}"

    at_pick = face_the_cup_lands_on(upright)
    at_handoff = face_the_cup_lands_on(yawed(upright, 180.0))
    assert at_pick != at_handoff, (
        f"both arms end up holding the part's {at_pick} face, so the second check "
        "re-inspects what the first one already saw")
    assert {at_pick, at_handoff} == {"+y", "-y"}, (
        f"the arms hold {at_pick} and {at_handoff}; they should be the two flat sides")


def test_an_unturned_handoff_would_leave_both_arms_on_the_same_face():
    """The control, so the test above cannot pass for the wrong reason."""
    from qc.round import _quat_conj, _rotate_by_quat, yawed

    upright = (1.0, 0.0, 0.0, 0.0)

    def face_the_cup_lands_on(part_wxyz):
        body = _rotate_by_quat(_quat_conj(tuple(part_wxyz)), (0.0, 1.0, 0.0))
        axis = max(range(3), key=lambda i: abs(body[i]))
        return f"{'+' if body[axis] > 0 else '-'}{'xyz'[axis]}"

    assert face_the_cup_lands_on(upright) == face_the_cup_lands_on(yawed(upright, 0.0))


# The trays used to sit 23 mm apart, which passes any test written at the tray centre and
# fails the moment an arm is in the picture: placing in one put arm-b's wrist inside the
# other, and the planner refused every IK solution. Both arms hold the part by its +y
# face throughout - that is fixed by the belt at the pick and by the half turn at the
# handoff - so every place is approached from +y, and the tray on the far side has to be
# clear of where that leaves the arm.

WRIST_REACH_MM = 135.0   # how far the ee_link capsule runs back along the tool from the flange
WRIST_RADIUS_MM = 40.0   # its radius, from the kinematics file
LINK_RADIUS_MM = 50.0    # the forearm's, likewise


# Measured on the machine at a stalled place, not read off a drawing: with the flange
# at (664, 572, 513) the wrist_1 link's origin sat at (669, 675, 417). So the wrist
# trails the flange by about 270 mm in +y - the direction the tool points back along -
# and hangs about 100 mm below it, and its own capsule is another 115 mm long.
WRIST_TRAIL_MM = 275.0
WRIST_DROP_MM = 100.0
WRIST_BELOW_MM = 215.0   # drop plus half the capsule


def wrist_box(flange_mm):
    """The box the wrist really occupies when the flange is here, gripping a +y face."""
    return (
        (flange_mm[0] - WRIST_RADIUS_MM, flange_mm[0] + WRIST_RADIUS_MM),
        (flange_mm[1] - WRIST_RADIUS_MM, flange_mm[1] + WRIST_TRAIL_MM),
        (flange_mm[2] - WRIST_BELOW_MM, flange_mm[2] + WRIST_RADIUS_MM),
    )


def spans(box):
    return tuple((box.centre_mm[i] - box.dims_mm[i] / 2.0,
                  box.centre_mm[i] + box.dims_mm[i] / 2.0) for i in range(3))


def overlap(a, b) -> bool:
    return all(lo_a < hi_b and lo_b < hi_a
               for (lo_a, hi_a), (lo_b, hi_b) in zip(a, b))


def place_flange(cell, tray):
    """Where arm-b's flange goes to lay the part on a conveyor."""
    from qc.round import nominal_place_flange

    return nominal_place_flange(cell, tray)


def test_the_place_puts_the_flange_behind_the_part_not_beside_it():
    """The property the whole outfeed layout rests on.

    Beside it - which is what a +y grip gives - the wrist trails about 275 mm across
    y, over whichever conveyor sits next to the target, and the arm fouls it. Four
    different conveyor arrangements were tried that way and each one found a different
    link hitting a different deck, because the planner picks a different configuration
    for each layout and a clearance fitted to one is wrong for the next.
    """
    cell = Cell.from_fragment()
    for tray in ("good", "bad"):
        flange, _ = place_flange(cell, tray)
        centre = cell.outfeed_point_mm(tray, cell.RELEASE_GAP_MM)
        assert flange[0] < centre[0] - 100.0, (
            f"the {tray} place puts the flange at x={flange[0]:.0f}, not behind the "
            f"part at {centre[0]:.0f}")
        assert abs(flange[1] - centre[1]) < 1.0, (
            "the flange should be in line with the part, not offset across it")


def test_the_place_keeps_the_whole_arm_off_both_decks():
    """With the flange behind the part, nothing of the arm is over a deck.

    Checked as the weakest claim that still means something: the flange, and therefore
    everything behind it, is short of where the conveyors start.
    """
    cell = Cell.from_fragment()
    for tray in ("good", "bad"):
        flange, _ = place_flange(cell, tray)
        for other in ("good", "bad"):
            deck = cell.trays[other]
            # Height, not plan position. Keeping the flange SHORT of the deck in x is
            # unsatisfiable once the part has to rest fully on it: the flange sits 175 mm
            # behind the part, so "part fully on" wants place_x >= 947 and "flange clear"
            # wants place_x < 785. The decks are 330 mm now and the arm works above them,
            # so what matters is that nothing of the arm comes DOWN onto one.
            assert flange[2] - WRIST_RADIUS_MM > deck.top_mm + 40.0, (
                f"placing on {tray} brings the wrist within "
                f"{flange[2] - WRIST_RADIUS_MM - deck.top_mm:.0f} mm of the {other} "
                f"conveyor's deck")


def test_the_place_points_the_tool_the_way_the_arm_reaches():
    """Radial, not tangential - which is what took the arm off the decks."""
    from qc.round import _ov_to_quat, _rotate_by_quat

    cell = Cell.from_fragment()
    base = cell.arm_bases_mm["arm-b"]
    for tray in ("good", "bad"):
        flange, orientation = place_flange(cell, tray)
        tool = _rotate_by_quat(_ov_to_quat(**orientation), (0.0, 0.0, 1.0))
        out = math.dist((flange[0], flange[1]), (base[0], base[1]))
        radial = ((flange[0] - base[0]) / out, (flange[1] - base[1]) / out, 0.0)
        off = math.degrees(math.acos(max(-1.0, min(1.0, sum(
            t * r for t, r in zip(tool, radial))))))
        assert off < 30.0, (
            f"the {tray} place points the tool {off:.0f} degrees off the direction "
            f"arm-b is reaching")


def test_the_outfeeds_run_along_x_not_along_y():
    """The direction is the whole reason the place works.

    Arm-b's wrist trails 270 mm behind the flange in +y. A conveyor running along y is
    underneath that wrist for the entire place; running along x it is only 180 mm wide
    in y and the wrist trails clear past it. This was found the expensive way, at three
    conveyor positions and two deck heights, all of which stalled.
    """
    cell = Cell.from_fragment()
    for tray in ("good", "bad"):
        deck = cell.trays[tray]
        assert deck.dims_mm[0] > deck.dims_mm[1] * 3, (
            f"the {tray} conveyor runs along y; its wrist clearance depends on it "
            f"running along x")


def test_the_place_is_at_the_same_reach_on_both_conveyors():
    """Both places share an x, so both use a configuration the cell has demonstrated."""
    cell = Cell.from_fragment()
    xs = [cell.outfeed_point_mm(t)[0] for t in ("good", "bad")]
    assert abs(xs[0] - xs[1]) < 1.0


def test_the_trays_do_not_overlap_each_other():
    cell = Cell.from_fragment()
    assert not overlap(spans(cell.trays["good"]), spans(cell.trays["bad"]))


# ---- defects -----------------------------------------------------------------

def part_prop():
    import json
    from pathlib import Path
    fragment = json.loads(
        (Path(__file__).resolve().parents[1] / "fragments" / "qc-cell.json").read_text())
    world = next(c for c in fragment["components"] if c["model"].endswith(":world"))
    return next(p for p in world["attributes"]["props"] if p["name"] == "part")


def part_children(name_prefix=""):
    return [c for c in part_prop().get("children", [])
            if c["name"].startswith(name_prefix)]


def part_is_asset() -> bool:
    """Whether the part is a referenced asset rather than a cube with children.

    The two carry their marks completely differently - a cube spawns one child per mark
    and hides them, an asset switches a variant set - so the checks that are about the
    MECHANISM differ, while the checks that are about the CELL (a mark on the face arm-a
    covers, a mark big enough to resolve) hold for both and are asserted for both.
    """
    return str(part_prop().get("type", "cube")) == "usd"


def test_defects_carry_no_collider():
    """A mark is paint.

    USD visibility is a render attribute and PhysX keeps the collider either way, so a
    defect spawned with a collider is a permanent obstacle whether or not it is showing -
    it would sit in the other arm's approach and the suction raycast would hit it.

    An asset's marks avoid the trap by construction: they are variant options, and an
    unselected variant's geometry is not composed into the stage at all, so there is
    nothing for PhysX to keep. That is strictly safer than `collision: false` and is why
    this check only has cube children to make.
    """
    if part_is_asset():
        pytest.skip("the asset switches marks by variant, so an unshown mark has no prim")
    for defect in part_children("defect_"):
        assert defect.get("collision") is False, (
            f"{defect['name']} would collide even while invisible")


def test_structural_children_still_collide():
    """The distinction matters in both directions: the part itself must be solid."""
    if part_is_asset():
        pytest.skip("the asset's shell and door are rigid bodies in the usd, not children")
    structural = [c for c in part_children() if not c["name"].startswith("defect_")]
    assert structural, "the part has no structural children"
    for child in structural:
        assert child.get("collision", True) is not False, (
            f"{child['name']} is part of the part and must collide")


def test_a_defect_sits_on_the_face_arm_a_covers():
    """Otherwise the second check can never change a verdict.

    arm-a's cup lands on the part's +y face - the belt leaves no other approach - so a
    cell whose defects all sit elsewhere would pass a test for 'defective goes to the
    bad tray' on arm-a's check alone, and the handoff would be decoration.
    """
    # Asked of the same derivation the cell uses, so it holds for either kind of part:
    # a cube's marks are placed by the fragment, an asset's were measured off the stage,
    # and both end up as a face per mark.
    # What matters is that no mark is PERMANENTLY hidden. The handoff in this cell is
    # forced by reach - arm-a cannot deliver to either outfeed - not by needing a second
    # look, so a defect does not have to hide under arm-a's cup to justify it. A defect
    # ON the gripped face, though, could never be seen by anything.
    cell = Cell.from_fragment()
    faces = Cell.defect_faces()
    assert faces, "the part has no defects at all"
    under_the_cup = [n for n, face in faces.items() if face == cell.GRIP_SIDE]
    assert not under_the_cup, (
        f"{under_the_cup} sit on {cell.GRIP_SIDE}, the face the cup covers for the whole "
        f"round - nothing could ever see them")


def test_defects_are_big_enough_to_see():
    """A mark the oracle knows about and no camera could resolve is not a defect."""
    if part_is_asset():
        import json
        derived = json.loads(
            (Path(__file__).resolve().parents[1] / "fragments"
             / "mailbox-derived.json").read_text())
        sizes = derived.get("decal_dims_mm") or {}
        assert sizes, "the asset's marks were never measured; run probes/derive_mailbox.py"
        for name, dims in sizes.items():
            # In its own plane: a decal is flat, so its thinnest axis is the surface
            # normal and says nothing about whether the mark can be seen.
            across = max(dims)
            assert across >= 15.0, (
                f"{name} is only {across:.0f} mm across, which at this cell's 1.35 px/mm "
                f"is {across * 1.35:.0f} px - too small to ask a detector for")
        return
    for defect in part_children("defect_"):
        largest = max(defect["scale"]) * 1000.0
        assert largest >= 25.0, (
            f"{defect['name']} is only {largest:.0f} mm across its longest side")




# ---- how comfortable the stations are ----------------------------------------
#
# "The trajectories look weird" is a real observation and a useless bug report until it
# is a number. These are the three numbers that were wrong: every presentation sat
# 435 mm above the arm's base, the worst station needed the base swung 47 degrees away
# from the direction the arm faces, and the arm reached 768 mm to get there. Rebuilding
# the cell around conveyors moved all three, and these hold them.

# How far a station may sit off the arm's forward direction. 30 everywhere the cell has a
# choice; 35 because one station does not.
#
# The handoff's give is 34.6 degrees off arm-a's forward direction and cannot be brought
# inside 30. The two ends of the handoff pull the fixture in opposite directions along x:
# arm-a gives the part by the face it carried, so its flange sits 212 mm SHORT of the
# fixture and wants the fixture far out to square up; arm-b takes the part by an end, so
# its flange sits at the fixture and wants it near. Measured across the range, the give
# only reaches 30 degrees at a fixture x of 889, and there arm-b's take is 894 mm out,
# past the 800 mm comfort bound and 88% of the arm's hard limit. x 780 is the best of it:
# give pan 34.6 and reach 699, take pan -0.2 and reach 786.
#
# Stated as a bound rather than an exception for one station, because the honest version
# of "this cell swings a base 35 degrees" is a cell-wide number that a reader can see.
MAX_PAN_DEG = 35.0
MAX_UP_MM = 280.0       # how far above the arm's base a station may sit
# And how far out. This is a comfort bound, not a feasibility one - `reach_budget()`
# is 915 mm and the measured hard limit 1016.7 - but a station near full extension is
# where a spherical bound stops being honest about orientation, and where the arm looks
# strained. The far outfeed conveyor is the one that sets it, at 791 mm.
# Raised from 800, which the far outfeed used to set at 791. The place now sets it: it has
# to happen downstream of the fixture, because the fixture straddles the outfeed lane and
# neither it nor the lane can move (see PLACE_PART_X_MM), and from there the two trays are
# 892 and 902 mm out.
#
# Set to `reach_budget()` rather than to a rounder number, so the cell has ONE outer bound
# instead of a comfort bound and a feasibility bound a few millimetres apart. The place is
# now within 13 mm of it.
#
# Worth saying plainly rather than burying in a constant: this cell is at its limits. The
# pan bound went 30 to 35 for the handoff's give and this one 800 to 930 for the place,
# each because the alternative measured worse, and the margins that remain between the
# infeed, the fixture, the inspection swing and the outfeed lanes are 20 to 60 mm. The
# next thing that moves needs all of them re-derived, not nudged.
MAX_REACH_MM = 915.0


def every_station():
    cell = Cell.from_fragment()
    for name, (arm, position) in cell.stations().items():
        yield cell, name, arm, position


def test_no_station_makes_an_arm_reach_across_its_own_body():
    for cell, name, arm, position in every_station():
        base = cell.arm_bases_mm[arm]
        pan = math.degrees(math.atan2(position[1] - base[1], position[0] - base[0]))
        assert abs(pan) <= MAX_PAN_DEG, (
            f"{name} sits {pan:.0f} degrees off {arm}'s forward direction; the arm has "
            f"to swing its base across itself to reach it")


def test_no_station_is_high_above_the_arms_base():
    """A station up near the shoulder is what makes an arm rear up over itself."""
    for cell, name, arm, position in every_station():
        up = position[2] - cell.arm_bases_mm[arm][2]
        assert up <= MAX_UP_MM, (
            f"{name} is {up:.0f} mm above {arm}'s base")


def test_no_station_is_near_full_extension():
    for cell, name, arm, position in every_station():
        reach = cell.reach_mm(arm, position)
        assert reach <= MAX_REACH_MM, f"{name} is {reach:.0f} mm from {arm}"


def test_the_pick_is_square_in_front_of_the_arm_that_makes_it():
    """The infeed conveyor exists to put the part here, so this one should be exact."""
    # Measured on the PART, which is what the infeed positions, not on the flange. The
    # flange sits well off arm-a's centreline by design: the grip is on a side face, so
    # the arm stands beside the part rather than behind it. Holding the FLANGE to 3
    # degrees would be asking the infeed to compensate for the choice of grip face.
    cell = Cell.from_fragment()
    base = cell.arm_bases_mm["arm-a"]
    part = cell.pick_mm
    pan = math.degrees(math.atan2(part[1] - base[1], part[0] - base[0]))
    assert abs(pan) <= MAX_PAN_DEG, (
        f"the part arrives {pan:.1f} degrees off arm-a's centreline")


def test_both_outfeed_conveyors_are_reached_without_swinging_the_base():
    cell = Cell.from_fragment()
    for tray in ("good", "bad"):
        flange, _ = cell.place(tray, PICK_SIDE, cell.RELEASE_GAP_MM)
        base = cell.arm_bases_mm["arm-b"]
        pan = math.degrees(math.atan2(flange[1] - base[1], flange[0] - base[0]))
        assert abs(pan) < 25.0, (
            f"the {tray} conveyor sits {pan:.0f} degrees off arm-b's centreline")


def test_the_outfeed_place_is_on_the_conveyor_not_at_its_centre():
    """A conveyor is metres long and its centre is nowhere near the arm.

    Deriving the place from the prop's centre - which is what the trays did, because a
    tray's centre IS its place - would put it out of reach the moment a tray became a
    conveyor.
    """
    cell = Cell.from_fragment()
    for tray in ("good", "bad"):
        point = cell.outfeed_point_mm(tray)
        deck = cell.trays[tray]
        near = deck.centre_mm[0] - deck.dims_mm[0] / 2.0
        far = deck.centre_mm[0] + deck.dims_mm[0] / 2.0
        assert near <= point[0] <= far, "the place is not on the conveyor at all"
        assert point[0] < deck.centre_mm[0], "the place should be at the near end"
        assert abs(point[1] - deck.centre_mm[1]) < 1.0, "and centred across it"


def test_the_part_is_inspected_above_the_outfeed_conveyors():
    """arm-b inspects the part directly over the conveyors, so it inspects above them."""
    cell = Cell.from_fragment()
    bottom = cell.inspect_point_mm("arm-b")[2] - cell.part.dims_mm[2] / 2.0
    for tray in ("good", "bad"):
        assert bottom > cell.trays[tray].top_mm + 100.0, (
            f"the part is inspected {bottom - cell.trays[tray].top_mm:.0f} mm over the "
            f"{tray} conveyor's deck")


def test_the_centre_table_is_low_enough_to_carry_the_part_over():
    """Both arms cross the middle of the cell with the part held at inspection height."""
    cell = Cell.from_fragment()
    lowest = cell.inspect_point_mm("arm-a")[2] - cell.part.dims_mm[2] / 2.0
    assert cell.fixture.top_mm < lowest - 50.0, (
        f"the centre table's top is {cell.fixture.top_mm:.0f} mm and the part passes "
        f"over it at {lowest:.0f} mm")


def test_taking_from_the_centre_table_clears_the_outfeed_decks():
    """arm-b reaches into the centre table with its wrist near deck height.

    The table sits low so the arms can carry the part over the middle of the cell, which
    puts the wrist down where the conveyors are. What keeps it out of them is that they
    start further out in x than the table.
    """
    cell = Cell.from_fragment()
    flange = cell.fixture_grip(PICK_SIDE)[0]
    for tray in ("good", "bad"):
        assert not overlap(wrist_box(flange), spans(cell.trays[tray])), (
            f"taking from the table puts arm-b's wrist through the {tray} conveyor")


def test_the_centre_table_clears_the_conveyor_decks():
    """Reaching into the table must not mean reaching through a deck.

    Checked in all three axes. The first version of this compared only y and z and
    failed on a wrist that was 30 mm clear in x - the same mistake as testing the
    handoff at the tray centre, one axis later.
    """
    cell = Cell.from_fragment()
    for label, pose in (("give", cell.fixture_grip("-y", cell.RELEASE_GAP_MM)),
                        ("take", cell.fixture_grip("+y"))):
        flange = pose[0]
        reach_y = (WRIST_REACH_MM if label == "take" else -WRIST_REACH_MM)
        box = (
            (flange[0] - WRIST_RADIUS_MM, flange[0] + WRIST_RADIUS_MM),
            tuple(sorted((flange[1] - WRIST_RADIUS_MM, flange[1] + reach_y))),
            (flange[2] - WRIST_RADIUS_MM, flange[2] + WRIST_RADIUS_MM),
        )
        for tray in ("good", "bad"):
            assert not overlap(box, spans(cell.trays[tray])), (
                f"the {label} wrist goes through the {tray} conveyor")


def test_the_second_check_is_made_clear_of_the_outfeed_decks():
    """arm-b inspects the part directly over the mouths of both conveyors.

    Its wrist runs 135 mm back from the flange along the tool, which at the inspection
    height put it 9 mm into a deck - and the planner refused every IK solution for two
    of the five poses. It is not enough for the PART to clear the conveyors; the arm
    holding it has to as well, and the arm is a great deal bigger than the part.
    """
    cell = Cell.from_fragment()
    for face, (flange, orientation) in cell.present_poses("arm-b"):
        along = WRIST_REACH_MM if orientation.get("o_y", 0.0) < 0 else WRIST_RADIUS_MM
        box = (
            (flange[0] - WRIST_RADIUS_MM, flange[0] + WRIST_RADIUS_MM),
            (flange[1] - WRIST_RADIUS_MM, flange[1] + along),
            (flange[2] - WRIST_RADIUS_MM, flange[2] + WRIST_RADIUS_MM),
        )
        for tray in ("good", "bad"):
            assert not overlap(box, spans(cell.trays[tray])), (
                f"showing the {face} face puts arm-b's wrist through the {tray} "
                f"conveyor")


def test_the_outfeed_decks_sit_below_the_working_height():
    """They are worked over, not just next to."""
    cell = Cell.from_fragment()
    working = cell.inspect_point_mm("arm-b")[2] - cell.grip_drop_mm - WRIST_RADIUS_MM
    for tray in ("good", "bad"):
        assert cell.trays[tray].top_mm < working - 30.0, (
            f"the {tray} conveyor's deck is at {cell.trays[tray].top_mm:.0f} mm and "
            f"arm-b works down to {working:.0f} mm over it")


def test_the_place_approach_clears_the_other_conveyors_deck():
    """The outfeeds are stepped, so approaching the low one passes over the high one.

    The cup hangs 145 mm off the flange along the grip axis, and at the approach height
    that put it through the high deck by 6 mm - enough for the planner to refuse every
    IK solution. The clearance the place descends from has to be chosen against the
    other conveyor, not just against the one being placed on.
    """
    from qc.round import PLACE_CLEAR_MM

    cell = Cell.from_fragment()
    reach = cell.tool_mm + cell.cup_radius_mm
    for tray, other in (("good", "bad"), ("bad", "good")):
        flange, _ = cell.place(tray, PICK_SIDE, cell.RELEASE_GAP_MM)
        above = (flange[0], flange[1], flange[2] + PLACE_CLEAR_MM)
        tool = (
            (above[0] - cell.cup_radius_mm, above[0] + cell.cup_radius_mm),
            (above[1] - reach, above[1] + cell.cup_radius_mm),
            (above[2] - cell.cup_radius_mm, above[2] + cell.cup_radius_mm),
        )
        assert not overlap(tool, spans(cell.trays[other])), (
            f"approaching the {tray} conveyor drags the cup through the {other} one")


def test_the_free_face_is_shown_from_its_own_position():
    """The four rolls and the free flat side are not the same kind of pose.

    The rolls turn the part where it hangs, so the arm stays put and stands off on its own
    side. The free side can only be shown with the arm BETWEEN the part and the camera,
    because the camera looks back along -x - so it needs its own station, and shown from
    the rolls' it was the pose that failed more than any other in this cell, in every
    layout it has had.

    What this used to assert was that the free station sits nearer the arm in y. That
    stopped being the right test when the rolls moved to the camera axis: for arm-a the
    two are now the same y, and what buys this pose its comfort is 200 mm of extra x
    rather than less y. So what is pinned is the property that was always the point - the
    free station is its own, and the arm is not folded under itself to reach it.
    """
    cell = Cell.from_fragment()
    for arm in ("arm-a", "arm-b"):
        rolls, free = cell.inspect_point_mm(arm), cell.free_point_mm(arm)
        assert free != rolls, f"{arm} shows the free face from the rolls' own station"
        flange = dict(cell.present_poses(arm))[
            "+y" if cell.present_side(arm) == "-y" else "-y"][0]
        reach = cell.reach_mm(arm, flange)
        assert 400.0 < reach <= MAX_REACH_MM, (
            f"{arm} reaches {reach:.0f} mm for the free face; under 400 it is folded "
            f"under itself, which is how this pose used to fail")


def test_every_presentation_the_camera_has_to_frame_is_in_frame():
    """Including the free positions, which are further apart than the rolls."""
    import json

    cell = Cell.from_fragment()
    config = json.loads(
        (Path(__file__).resolve().parents[1] / "fragments" / "qc-cell.json").read_text())
    camera = next(c for c in config["components"] if c["name"] == "inspect-cam")
    distance = math.dist(cell.camera_mm, cell.camera_target_mm)
    half_frame = distance * math.tan(
        math.radians(float(camera["attributes"]["fov_deg"])) / 2.0)
    silhouette = math.dist((0, 0, 0), cell.part.dims_mm) / 2.0
    for arm in ("arm-a", "arm-b"):
        for label, point in (("rolls", cell.inspect_point_mm(arm)),
                             ("free", cell.free_point_mm(arm))):
            edge = abs(point[1] - cell.camera_target_mm[1]) + silhouette
            assert edge < half_frame, (
                f"{arm}'s {label} position puts the part's edge {edge:.0f} mm off "
                f"axis, outside the {half_frame:.0f} mm half-frame")


# ---- the part's own size -----------------------------------------------------


def test_part_local_dims_match_the_assets_own_collision_bounds():
    """The part's size in its own frame, cross-checked against the asset.

    `part.dims_mm` is a world-aligned extent measured at the configured yaw, and
    `part_local_dims_mm` turns it back. That arithmetic has an independent witness: the
    mailbox publishes `collisionBoundsMin/Max` as USD attributes, in its OWN frame, and
    they are not involved in deriving either number. If the swap is right they agree.

    Worth pinning because the version of this that read the live pose cost all five of
    arm-a's presentations: held at the 16 degree shoulder tilt the part measured 55 and
    42 mm larger than it is, and the planner refused every pose with the carried box
    inside the belt deck.
    """
    from qc.layout import Cell, asset_facts

    cell = Cell.from_fragment()
    truth = asset_facts()["ground_truth"]
    low = truth["mailbox:collisionBoundsMin"]
    high = truth["mailbox:collisionBoundsMax"]
    own = [(high[i] - low[i]) * 1000.0 for i in range(3)]
    for axis, (derived, stated) in enumerate(zip(cell.part_local_dims_mm, own)):
        assert derived == pytest.approx(stated, abs=0.5), (
            f"axis {axis}: the layout derives {derived:.1f} mm from the world extent "
            f"and the asset states {stated:.1f} mm in its own frame")


def test_a_part_yawed_off_the_right_angles_is_refused():
    """A world extent at, say, 30 degrees is strictly larger than the part and cannot be
    turned back into it. Refused rather than approximated, because the approximation is
    exactly the silent inflation that broke the presentations."""
    from qc.layout import _local_dims

    assert _local_dims((172.4, 494.9, 226.9), -90.0) == (494.9, 172.4, 226.9)
    assert _local_dims((494.9, 172.4, 226.9), 180.0) == (494.9, 172.4, 226.9)
    with pytest.raises(ValueError, match="multiple of 90"):
        _local_dims((400.0, 300.0, 226.9), 30.0)


def test_the_origin_offset_in_the_parts_own_frame_matches_the_asset():
    """The origin-to-centre offset, turned back into the part's frame.

    It has to be a vector in the PART, because `part_pose` rotates it by the part's live
    orientation to report a centre. Stored, it is a world-frame offset at the configured
    yaw, and added unrotated it put a rolled part's centre up to 151 mm from where the
    part was - which read as the cup sliding 149 mm over a presentation roll.

    The witness is the asset's own `collisionBounds`, in its own frame, which neither
    derivation touches: the middle of those bounds IS the offset.
    """
    from qc.layout import asset_facts, part_origin_offset_local_mm

    truth = asset_facts()["ground_truth"]
    low = truth["mailbox:collisionBoundsMin"]
    high = truth["mailbox:collisionBoundsMax"]
    stated = [(low[i] + high[i]) / 2.0 * 1000.0 for i in range(3)]
    for axis, (derived, witness) in enumerate(
            zip(part_origin_offset_local_mm(), stated)):
        assert derived == pytest.approx(witness, abs=0.5), (
            f"axis {axis}: the layout derives {derived:.2f} mm by un-yawing the stored "
            f"offset and the asset's own bounds put it at {witness:.2f} mm")


# ---- what one part should cost -----------------------------------------------


def test_the_cycle_budget_is_derived_from_the_cells_own_distances():
    """A cycle time has to come from the geometry, not from a number someone liked.

    What is pinned is the SHAPE of the derivation, not the total: every leg's time is the
    trapezoidal time for that leg's measured distance at the stated speed, so moving a
    station changes the budget without anyone editing it.
    """
    from qc.cycle import budget, legs, move_seconds

    cell = Cell.from_fragment()
    b = budget(cell)
    speed, accel = cell.cycle["tcp_speed_mm_s"], cell.cycle["tcp_accel_mm_s2"]
    for arm, chain in legs(cell).items():
        travel = sum(move_seconds(d, speed, accel) for _, d in chain)
        assert b["per_arm_s"][arm] == pytest.approx(travel + b["handling_s"]), (
            f"{arm}'s budget is not its own travel plus the handling it does")
    assert b["serial_s"] == pytest.approx(sum(b["per_arm_s"].values()))
    assert b["pipelined_s"] == pytest.approx(max(b["per_arm_s"].values()))


def test_short_moves_are_not_costed_at_the_cruise_speed():
    """Most of this cell's moves never reach cruise - a 35 mm descent is all ramp - and
    costing them at distance/speed under-counts them about threefold."""
    from qc.cycle import move_seconds

    naive = 35.0 / 600.0
    assert move_seconds(35.0, 600.0, 1500.0) > 3.0 * naive
    # and a long move is dominated by the cruise, within the two ramps
    assert move_seconds(3000.0, 600.0, 1500.0) == pytest.approx(3000.0 / 600.0 + 0.4, abs=0.05)


def test_pipelining_is_the_bigger_lever_than_arm_speed():
    """Stated as a test because it is the cell's main throughput fact: overlapping the two
    arms nearly halves the cycle, which no realistic arm speed-up matches."""
    from qc.cycle import budget

    b = budget(Cell.from_fragment())
    assert b["pipelined_s"] < 0.6 * b["serial_s"]
