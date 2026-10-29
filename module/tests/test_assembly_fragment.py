"""The assembly cell's geometry and its generated fragment, checked without a machine.

This file is the guard `probes/assembly/build_fragment.py` has claimed since its first
commit - in its own docstring and in the `_comment` it writes into every fragment - and
which did not exist. Thirty commits of cell went by on the strength of an assertion that a
test enforced the generator. That is a worse state than having no test, because it told
anyone reading the fragment that a hand edit would be caught.

Everything here is arithmetic over the asset's published numbers and the generator's own
output, so it runs without Isaac. Each check is a fault this cell actually had; none of
them are hypotheses about what might go wrong.
"""

import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "probes"))

from assembly.colliders import pieces
from assembly.asset import insertions, part                       # noqa: E402
from assembly.build_fragment import FRAGMENT, build, chosen       # noqa: E402
from assembly.cell import MAX_PAN_DEG, MAX_UP_MM, REACH_BUDGET_MM  # noqa: E402
from assembly.fixture import (CARRIED, feed_grasp_mm, feed_spot_mm,  # noqa: E402
                              feeder_boxes, pin_corridor, saddle)
from assembly.stations import THREE, TOOLS, stations_for          # noqa: E402

# Imported from the module that does the clearing, so this checks THAT number rather
# than a copy of it.
from assembly.arms import CLEAR_BACKOFF_MM                        # noqa: E402

CELL = chosen()


def _sanitise(name: str) -> str:
    """What the module will turn a prop or child name into, as a usd prim name."""
    return "".join(c if c.isalnum() or c == "_" else "_" for c in name)


def _boxes(a, b):
    """Do two (centre, dims) boxes overlap on every axis?"""
    return all(abs(a[0][i] - b[0][i]) < (a[1][i] + b[1][i]) / 2.0 for i in range(3))


# ---- the guard the generator has been claiming --------------------------------------

def test_the_committed_fragment_is_what_the_generator_produces():
    """A hand edit to the json is the divergence vector; this is what closes it."""
    on_disk = json.loads(FRAGMENT.read_text())
    assert on_disk == build(CELL), (
        f"{FRAGMENT.name} differs from what build_fragment.py produces. Regenerate it "
        f"rather than editing it: the fragment is one of two consumers of the cell "
        f"description and a hand edit makes it a second source.")


# ---- the bug that cost the most ------------------------------------------------------

def test_no_two_prop_names_collide_once_sanitised():
    """`lip_-y` and `lip_+y` are the same usd prim, and one silently replaces the other.

    This had a wall on one side of the fixture and open air opposite it for the whole of
    this cell's life - the part walked 0.2 mm a cycle and three rounds of lip geometry
    could not stop it. It had already turned a two-rail saddle into a one-rail saddle,
    which was written up as "two narrow rails cannot hold a tall part" and was nothing of
    the kind.
    """
    props = build(CELL)["components"][0]["attributes"]["props"]
    seen = {}
    for prop in props:
        for scope, names in (("prop", [prop["name"]]),
                             (prop["name"], [c["name"] for c in prop.get("children", [])])):
            local = {}
            for name in names:
                key = _sanitise(name)
                assert key not in local, (
                    f"{scope}: {local[key]!r} and {name!r} both sanitise to {key!r}")
                local[key] = name
            seen.setdefault(scope, local)


# ---- the fixture has to fit the part --------------------------------------------------

def test_the_lips_clear_the_shell_envelope():
    """Lips inside the part are lips the part rides ON rather than between.

    Built to `collisionBounds` they were, because the shell's hinge knuckles carry no
    colliders - by design, so the pin can pass - and stand 5.7 mm in front of that box.
    """
    shell = part("Shell")
    low, high = shell.envelope_mm
    pieces = {b.name: b for b in saddle((CELL.index_x("inside"), CELL.belt_y_mm, 0.0),
                                        CELL.belt_top_mm, CELL.fixture_lift_mm)}
    for name in ("lip_my", "lip_py"):
        lip = pieces[name]
        inner = abs(lip.centre_mm[1]) - lip.dims_mm[1] / 2.0
        assert inner >= high[1], (
            f"{name}'s inner face is at |y| {inner:.2f}, inside the shell's envelope "
            f"half-width of {high[1]:.2f} - the part cannot sit between the lips")
    back = pieces["lip_back"]
    face = back.centre_mm[0] + back.dims_mm[0] / 2.0
    assert face <= CELL.index_x("inside") + low[0], (
        f"the back lip's face is at x {face:.2f}, inside the shell's envelope")


def test_the_pin_corridor_is_clear_of_the_fixture():
    """Nothing may stand in the volume the pin and its fingers sweep."""
    work = (CELL.index_x("pin"), CELL.belt_y_mm, 0.0)
    corridor = pin_corridor(work, CELL.belt_top_mm, CELL.fixture_lift_mm)
    for piece in saddle(work, CELL.belt_top_mm, CELL.fixture_lift_mm):
        assert not _boxes((corridor.centre_mm, corridor.dims_mm),
                          (piece.centre_mm, piece.dims_mm)), (
            f"{piece.name} stands in the pin corridor")


# The keep-out the mailbox asset's owner specified for the pin, in the pin's own frame:
# "keep supports at y <= about -0.010". Written here as the external requirement it is,
# so this test fails if either the placement or the clearance constant drifts off it.
PIN_SUPPORT_MAX_Y_MM = -10.0


# ---- a station has to be sited where the cycle actually stops ------------------------

def test_every_station_is_sited_at_the_index_its_cycle_step_uses():
    """A station sited for one index and visited at another aims at bare belt.

    Both pin stations were sited at the DOOR's index, 152.3, left behind when the door
    stations were deleted, while the cycle indexes its pin step to 363.8 - so the pick
    reached for a pin 211.5 mm away and caught nothing. It passed every run anyway,
    because `seat:Pin` calls `join`, and a join snaps the part onto its published frame
    whether or not an arm ever held it. The cell reported the pin seated 0.00 mm from its
    published position while the pin had not moved.
    """
    from assembly.film_cycle import CYCLE                          # noqa: PLC0415

    by_task = {st.task: st for st in stations_for(CELL)}
    for step, arm, pose, _tool, _what in CYCLE:
        if arm is None or pose is None:
            continue
        station = by_task.get(pose)
        assert station is not None, f"the cycle visits {pose!r}, which is not a station"
        if not station.on_belt:
            continue
        want = CELL.index_x(step)
        assert abs(station.index_x_mm - want) < 1e-6, (
            f"{pose} is sited for index {station.index_x_mm:.1f} but the cycle runs it "
            f"at step {step!r}, index {want:.1f} - "
            f"{abs(station.index_x_mm - want):.1f} mm from where the part stops")


def test_nothing_on_the_pallet_intrudes_into_the_shell_s_seat():
    """A cradle holds a part square only if nothing is propping it up.

    The shell was measured sitting about 15 degrees out of square at the pin index, which
    a cradle with 1.2 mm of clearance and 70 mm lips should not permit - so the first
    question is whether some other piece of the pallet is under it. The tray, its rests,
    its rails and the end stops all live on the same body as the cradle and have all been
    added or moved since the cradle was sized.
    """
    pieces = saddle((CELL.index_x("inside"), CELL.belt_y_mm, 0.0),
                    CELL.belt_top_mm, CELL.fixture_lift_mm)
    seat = {b.name for b in pieces
            if b.name.startswith(("saddle", "rail_rib", "lip"))}
    low, high = part("Shell").envelope_mm
    origin = CELL.shell_origin_mm
    box = [(origin[i] + low[i], origin[i] + high[i]) for i in range(3)]
    for b in pieces:
        if b.name in seat:
            continue                      # the cradle is meant to touch it
        overlap = [min(box[k][1], b.centre_mm[k] + b.dims_mm[k] / 2.0)
                   - max(box[k][0], b.centre_mm[k] - b.dims_mm[k] / 2.0)
                   for k in range(3)]
        assert min(overlap) <= 0.0, (
            f"{b.name} reaches into the shell's seated volume by "
            f"{[round(v, 1) for v in overlap]} mm in x/y/z, so the shell is resting on "
            f"it rather than on its cradle")


# ---- a suction cup has to stop SHORT of the surface ----------------------------------

def test_every_cup_stands_off_the_part_it_grips():
    """A cup landing on the face has nothing to reach for; one inside it cannot seal.

    This is the fault that passed for weeks. Two of the cell's three picks were holding
    air - the pin's cup landed at y 291.5 against a pin whose end face is y 291.5, and
    arm-c's landed 0.16 mm inside the shell's wall - and both logged as passes, because
    only motion completion was checked and the seating measurement that followed was of a
    part the `join` had snapped onto its published frame.

    The invariant is one number: a station's `extra_mm` beyond the surface it is reaching
    for. Where that surface sits relative to the station's work point differs by station
    and the station is the only thing that knows, so it is stated here rather than
    inferred - a pick's work point is the part's ORIGIN, and `grasp`'s is already a point
    ON the shell's wall.
    """
    # the gripper's reach, from the config rather than from a number copied into a test
    # - per gripper: arm-b's jaw reaches further than arm-c's cup.
    reach_by_arm = {f"arm-{c['name'][-1]}": float(c["attributes"]["max_grip_distance"]) * 1000.0
                    for c in json.loads(FRAGMENT.read_text())["components"]
                    if c["type"] == "gripper"}
    for station in stations_for(CELL):
        if station.tool not in ("jaw", "big"):
            continue
        if station.task.endswith("_pick"):
            # HOW FAR THE CUP STOPS FROM THE FACE IT ACTUALLY MEETS, measured against the
            # part's colliders rather than inferred from its published grasp width.
            #
            # The width proxy assumed a CROSSWISE approach: the cup meets the part's flank,
            # so the surface is half a grasp-width from the work point. That stopped being
            # true when the pin's pick came round to +Y to match its insert - along its own
            # axis the cup meets the grip collar's END face, 10 mm from the grasp point,
            # not its Ø7.4 flank 3.7 mm away. The test fired, correctly, on a station that
            # was right; what was wrong was the model it was checked against.
            #
            # Measuring the collider the grasp point sits in covers both cases with one
            # rule, and it is a stronger check than the proxy: `graspWidth` is a number the
            # asset publishes about itself, while this is the geometry the gripper's
            # raycast will actually hit.
            which = station.task.split("_")[0].capitalize()
            grasp = [v * 1000.0 for v in part(which).at("assembly:graspPosition")]
            face = None
            for piece in pieces(which):
                if all(piece["min_mm"][i] - 0.01 <= grasp[i] <= piece["max_mm"][i] + 0.01
                       for i in range(3)):
                    face = sum(station.approach[i]
                               * ((piece["max_mm"][i] if station.approach[i] > 0
                                   else piece["min_mm"][i]) - grasp[i]) for i in range(3))
                    break
            assert face is not None, (
                f"{station.task}: {which}'s published grasp point is not inside any of "
                f"its colliders, so there is nothing there to grip")
            reach_past = station.extra_mm - face
        elif station.task == "grasp":
            reach_past = station.extra_mm      # work point IS the wall
        else:
            continue
        assert reach_past >= 1.0, (
            f"{station.task}: the cup stops {reach_past:+.2f} mm from the surface - "
            f"flush or inside it, so there is nothing to seal against")
        grip_mm = reach_by_arm[THREE[station.role]]
        assert reach_past <= grip_mm, (
            f"{station.task}: the cup stops {reach_past:.1f} mm short, past the "
            f"gripper's {grip_mm:.0f} mm reach - it will never attach")


# ---- the jaws have to close around the part, not in front of it ----------------------

def _tool_spec():
    for c in build(CELL)["components"]:
        spec = (c.get("attributes") or {}).get("jaw")
        if isinstance(spec, dict):
            return spec
    raise AssertionError("no gripper in the fragment declares a tool")


def test_both_arms_carry_a_real_tool_asset_and_only_one_each():
    """A suction cup with cuboid fingers drawn over it is not a machine.

    Both grippers used to declare one: a cup doing the holding and two boxes drawn on top,
    on BOTH arms. That cannot be built - a cup cannot grip the end of an 8 mm pin and two
    fingers cannot span a 167 mm shell - and it is what made the cell read as fake.
    """
    tools = {c["name"]: (c["attributes"].get("jaw") or {})
             for c in build(CELL)["components"] if c["type"] == "gripper"}
    assert tools, "no grippers in the fragment"
    for name, spec in tools.items():
        assert spec.get("usd_path"), (
            f"{name} has no tool asset - it is still a stand-in")
        assert Path(spec["usd_path"]).exists(), (
            f"{name}'s tool asset is not on disk: {spec['usd_path']} - mirror it, "
            f"because streaming it at boot corrupts physics")
        # A tool either has fingers that move or it does not; nothing here has both a
        # cup and a set of jaws.
        assert "fingers" in spec


def test_the_picks_take_each_part_where_the_ASSET_says_to():
    """Derived from the published grasp, not from a bounding box.

    A bounding box is how this cell came to aim a 22 mm cup at the 8 mm end face of a pin
    and call it a pick. The part says where it is held and how wide it is there; the cell
    says only which way to come at it, because that depends on the tray and the asset
    knows nothing about the tray.
    """
    for station in stations_for(CELL):
        if not station.task.endswith("_pick"):
            continue
        which = station.task.split("_")[0].capitalize()
        p = part(which)
        want = feed_grasp_mm(CELL, which)
        for i in range(3):
            assert abs(station.work_mm[i] - want[i]) < 1e-6, (
                f"{station.task} works at {station.work_mm}, which is not the part's "
                f"published graspPosition where its feeder presents it")
        assert not station.on_belt, f"{station.task} is a fixed feeder, not a belt point"
        # and the jaw has to be able to open around it
        width = float(p.at("assembly:graspWidth")) * 1000.0
        assert 0.0 < width <= float(_tool_spec()["open_mm"]), (
            f"{which} is {width:.1f} mm across its grasp and the tool opens "
            f"{_tool_spec()['open_mm']:.1f}")


def test_no_pick_puts_the_flange_under_the_belt():
    """The published approach is about the SEATED pose; the tray is a different place.

    The pin publishes graspApproach (0,0,1) - from below - because once it is seated the
    shell floor is 2.7 mm above it. Applied literally at the pick that put arm-b's flange
    at z 583, which is 67 mm under the conveyor deck.
    """
    deck = CELL.belt_top_mm
    for station in stations_for(CELL):
        if not station.task.endswith("_pick"):
            continue
        assert station.flange_mm()[2] > deck, (
            f"{station.task} puts the flange at z {station.flange_mm()[2]:.1f}, under "
            f"the deck at {deck:.1f}")


# ---- the conveyor is an asset now, with the asset's own obstacles ---------------------

def test_the_deck_covers_every_index_the_cycle_uses():
    """The line is several 1999 mm segments butted end to end; a station off the end of
    them has no deck under the pallet at all."""
    props = {p["name"]: p for p in build(CELL)["components"][0]["attributes"]["props"]}
    belt = props["belt"]
    deck = next(b for b in belt["bounds_m"] if b["name"] == "deck")
    segs = [p for n, p in props.items() if n == "belt" or n.startswith("belt_")]
    lo = min(p["position"][0] + deck["min"][0] for p in segs) * 1000.0
    hi = max(p["position"][0] + deck["max"][0] for p in segs) * 1000.0
    for name, x in CELL.index_x_for.items():
        assert lo + 200.0 <= x <= hi - 200.0, (
            f"index {name} at x {x:.1f} leaves the pallet less than 200 mm of deck "
            f"(deck {lo:.1f}..{hi:.1f})")
    top = (belt["position"][2] + deck["max"][2]) * 1000.0
    assert abs(top - CELL.belt_top_mm) < 0.05, (
        f"the deck's top is at z {top:.2f} and the cell places parts at "
        f"{CELL.belt_top_mm:.2f}: everything on the belt is {top - CELL.belt_top_mm:+.2f} "
        f"mm out")


def test_the_clearing_index_misses_the_conveyors_uprights():
    """`clear_and_park` runs the pallet downstream; the asset has posts down there."""
    props = {p["name"]: p for p in build(CELL)["components"][0]["attributes"]["props"]}
    belt = props["belt"]
    posts = [b for b in belt["bounds_m"] if b["name"].startswith("post")]
    assert posts, "the conveyor asset declares no uprights, so this check is vacuous"
    nearest = min((belt["position"][0] + b["min"][0]) * 1000.0 for b in posts)
    far = CELL.belt_to_x_mm - CLEAR_BACKOFF_MM
    # The furthest downstream thing the pallet carries, at that index.
    reach = max(b.centre_mm[0] + b.dims_mm[0] / 2.0 - CELL.index_x("inside")
                for b in saddle((CELL.index_x("inside"), CELL.belt_y_mm, 0.0),
                                CELL.belt_top_mm, CELL.fixture_lift_mm))
    assert far + reach <= nearest - 100.0, (
        f"clearing to x {far:.1f} puts the pallet's downstream edge at "
        f"{far + reach:.1f}, against the conveyor's uprights at {nearest:.1f} - under "
        f"100 mm for a 40 kg pallet pushed by friction")


# ---- the tray has to hold what it carries ---------------------------------------------
#
# These two were written against the standing feeder rack, which is gone: the parts now
# ride a tray on the pallet so the belt does not have to stop for a feeder. The faults
# they catch are the tray's faults too - a part over the edge still tips off, and the pin
# is still lifted along its own axis - so they are kept and re-aimed rather than deleted.

def _tray_box():
    """The tray's x/y footprint, from the composite piece the generator emits."""
    tray = next(b for b in saddle((CELL.index_x("inside"), CELL.belt_y_mm, 0.0),
                                  CELL.belt_top_mm, CELL.fixture_lift_mm)
                if b.name == "carrier")
    return tray.centre_mm, tray.dims_mm


# ---- the stations have to be reachable ------------------------------------------------

@pytest.mark.parametrize("station", [s.task for s in stations_for(CELL)])
def test_every_station_is_inside_the_sweep_budgets(station):
    st = next(s for s in stations_for(CELL) if s.task == station)
    arm = THREE[st.role]
    flange = st.flange_mm()
    reach = CELL.reach_mm(arm, flange)
    assert reach <= REACH_BUDGET_MM, f"{st.key}: reach {reach:.0f} > {REACH_BUDGET_MM:.0f}"
    assert CELL.up_mm(arm, flange) <= MAX_UP_MM, f"{st.key}: too high"


def test_a_camera_station_does_not_put_its_lens_on_the_work():
    """The fisheye's lens rides its stem's tip, so a surface work point is a contact print.

    The back-cap shot photographed the shell's unlit interior from inside its own back
    wall until it was given an optical stand-off.
    """
    for st in stations_for(CELL):
        if st.tool not in ("fisheye", "realsense"):
            continue
        gap = math.dist(st.lens_mm(), st.work_mm)
        if st.task == "inside":
            continue        # this one is deliberately 70 mm INSIDE the aperture
        assert gap >= 50.0, (
            f"{st.key}: the lens sits {gap:.0f} mm from the work point")


# ---- what the asset publishes, used as published --------------------------------------

def test_every_insertion_has_a_joint_the_cell_can_make():
    """Parts do not stay assembled because they are in the right place."""
    for name, ins in insertions().items():
        assert ins.joint in ("fixed", "revolute"), f"{name}: no seat joint"
        assert ins.depth_mm >= ins.engage_mm, (
            f"{name}: insertDepth {ins.depth_mm} is less than engageDepth "
            f"{ins.engage_mm}; the pre-insertion pose would start past first contact")
        if ins.joint == "revolute":
            assert ins.joint_drive, (
                f"{name}: a revolute seat joint with no drive is a free hinge - this "
                f"door's centre of mass sits 112.7 mm above its axis and it falls open")


def test_the_carried_set_is_what_the_fragment_actually_has():
    props = {p["name"] for p in build(CELL)["components"][0]["attributes"]["props"]}
    for name in CARRIED:
        assert name in props, f"the carried set names {name!r}, which is not a prop"


# ---- the feeders ------------------------------------------------------------------------

def test_the_pin_stands_on_its_nest_axis_with_its_grasp_straight_above_the_seat():
    """A three-jaw pick only centres the pin if its grasp is on the nest's axis - on the
    SHAFT just under the head since the flush pin (2026-10-13), the published distance
    below the pin's origin (its head)."""
    seat = feed_spot_mm(CELL, "Pin")
    grasp = feed_grasp_mm(CELL, "Pin")
    assert abs(grasp[0] - seat[0]) < 1e-3 and abs(grasp[1] - seat[1]) < 1e-3
    below = -part("Pin").at("assembly:graspPosition")[1] * 1000.0
    assert abs((seat[2] - grasp[2]) - below) < 1e-3


def test_the_feeders_stand_clear_of_the_main_belt():
    """Fixed boxes beside the belt must not reach over its guide rail (y 452..502)."""
    import math
    for name, centre, dims, yaw in feeder_boxes():
        a = math.radians(yaw)
        half = [dims[0] / 2.0, dims[1] / 2.0]
        reach = abs(math.sin(a)) * half[0] + abs(math.cos(a)) * half[1]
        # Overhanging the rail in plan is fine for a box that sits ON it: the rail's top
        # is z 670, and the flag feeder's deck corner rests there (the pick spot moved to
        # where arm-b reaches it on the insert's wrist branch).
        if centre[2] - dims[2] / 2.0 >= 669.5:
            continue
        assert centre[1] - reach > 505.0, (
            f"{name} reaches to y {centre[1] - reach:.1f}, over the main belt's guide")


def test_the_flag_rests_with_its_leading_end_at_the_stop():
    """The flag's pick is computed where the stop holds it; the stop has to be there."""
    import math
    stop = next(c for n, c, d, y in feeder_boxes() if n == "flag_feed_stop")
    grasp = feed_grasp_mm(CELL, "Flag")
    assert math.dist(stop[:2], grasp[:2]) < 60.0, (
        f"the flag's shaft waits {math.dist(stop[:2], grasp[:2]):.1f} mm from its stop")
