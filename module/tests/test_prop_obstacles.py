"""The obstacle list the planner depends on, checked against the cell's real props.

`SimManager.prop_obstacles()` is the only thing standing between the motion service and
planning straight through the scenery, and it is pure arithmetic over the config, so it
does not need isaac to be checked. What it has to get right is the box: a cube prop's
edge is `size`, scaled per axis, in metres, and viam's geometry messages are full
dimensions in millimetres. Getting `size` and `scale` the wrong way round, or reporting a
half-extent, would silently shrink or inflate every obstacle in the cell.
"""

import json
from pathlib import Path

import pytest

from isaac_module.sim_manager import SimConfig, SimManager

FRAGMENT = Path(__file__).resolve().parents[1] / "fragments" / "qc-cell.json"


@pytest.fixture
def configured():
    """The props exactly as the fragment declares them."""
    components = json.loads(FRAGMENT.read_text())["components"]
    world = next(c for c in components if c["model"].endswith(":world"))
    return {p["name"]: p for p in world["attributes"]["props"]}


@pytest.fixture
def obstacles():
    """Run the real config's props through the real method."""
    components = json.loads(FRAGMENT.read_text())["components"]
    world = next(c for c in components if c["model"].endswith(":world"))
    manager = SimManager.get()
    previous = manager.cfg
    manager.cfg = SimConfig(props=[dict(p) for p in world["attributes"]["props"]])
    try:
        yield {o["label"]: o for o in manager.prop_obstacles()}
    finally:
        manager.cfg = previous


def test_every_cube_prop_is_reported(obstacles, configured):
    """Every one, derived from the fragment.

    Naming the props here is what made this test fail the moment the trays became
    conveyors, which is drift of exactly the kind `stations.json` was deleted for. The
    fragment is the list; this only checks nothing falls out of it.
    """
    expected = {name for name, prop in configured.items()
                if str(prop.get("type", "cube")) == "cube"}
    assert expected <= set(obstacles), (
        f"not reported: {sorted(expected - set(obstacles))}")


def test_the_floor_is_reported(obstacles):
    """boot() adds a ground plane for any scene without its own stage.

    It is a collider like any other, and an arm on a pedestal has plenty of
    configurations that reach below its own base. Unreported, the planner routes an
    elbow into it and the arm stalls pressing on the floor - which looks like a
    mysterious failure to settle rather than a collision.
    """
    ground = obstacles["ground"]
    assert ground["fixed"]
    top = ground["center_mm"][2] + ground["dims_mm"][2] / 2
    assert top == pytest.approx(0.0), "the floor's top face should be z=0"
    assert min(ground["dims_mm"][:2]) > 5000, "the floor should span the whole cell"


def test_dimensions_are_full_extents_in_mm(obstacles, configured):
    """size * scale, in metres, reported as full extents in millimetres.

    Checked against the fragment's own numbers rather than a copy of them: getting
    `size` and `scale` the wrong way round, or reporting a half-extent, would silently
    shrink or inflate every obstacle in the cell.
    """
    for name, prop in configured.items():
        if str(prop.get("type", "cube")) != "cube" or name not in obstacles:
            continue
        size = float(prop.get("size", 0.05))
        scale = [float(v) for v in (prop.get("scale") or (1.0, 1.0, 1.0))]
        assert obstacles[name]["dims_mm"] == pytest.approx(
            [size * scale[i] * 1000.0 for i in range(3)]), name


def test_scale_applies_per_axis(obstacles):
    """A conveyor deck is long in y, narrow in x and thin in z - three different axes.

    A scale applied uniformly, or on the wrong axis, would still produce a plausible
    box, so the test needs a prop whose three extents genuinely differ.
    """
    # The outfeeds are referenced assets now, reported as several boxes - deck, walls,
    # legs - so the deck is addressed by its own label rather than the prop's.
    deck = obstacles["good_conveyor.deck"]
    assert deck["dims_mm"][0] > deck["dims_mm"][1] * 3, "an outfeed runs along x"
    assert deck["dims_mm"][2] < deck["dims_mm"][1] / 2, "and its deck is thin"


def test_fixed_is_carried_through(obstacles):
    """`fixed` is what tells a caller whether a prop is scenery or cargo.

    A fixed prop is an obstacle for the whole round. A dynamic one is an obstacle until
    an arm picks it up, and then becomes part of that arm.
    """
    # Matched by prefix: the belt is a real conveyor now and reports as `belt.deck`,
    # `belt.rail_l` and so on, so `"belt" in obstacles` is False and this test quietly
    # stopped checking the one prop most able to ruin a round by moving.
    scenery = {"belt", "good_conveyor", "bad_conveyor", "ground"}
    found = {n for n in scenery
             for label in obstacles if label == n or label.startswith(f"{n}.")}
    assert "belt" in found, "the belt must still be checked, whatever its labels"
    assert all(o["fixed"] for label, o in obstacles.items()
               if label.split(".")[0] in scenery), "the cell's scenery must be fixed"
    # The part is not scenery. It is an obstacle while it sits on the belt and becomes
    # part of the arm holding it, and `fixed` is the only thing that says which.
    assert obstacles["part"]["fixed"] is False, \
        "the part must be dynamic, or the gripper has nothing it can actually pick up"


def test_a_scene_with_its_own_stage_reports_no_floor():
    """A usd_stage brings its own floor, if it has one - do not invent a second."""
    manager = SimManager.get()
    previous = manager.cfg
    manager.cfg = SimConfig(usd_stage="/scene.usd", props=[])
    try:
        assert manager.prop_obstacles() == []
    finally:
        manager.cfg = previous


def test_no_props_means_no_obstacles():
    manager = SimManager.get()
    previous = manager.cfg
    manager.cfg = SimConfig(props=[])
    try:
        # The ground plane is still there - it is part of the scene, not a prop.
        assert [o["label"] for o in manager.prop_obstacles()] == ["ground"]
    finally:
        manager.cfg = previous


def test_a_usd_prop_without_an_extent_is_refused_rather_than_skipped():
    """A usd prop the planner cannot see is a fault, not a prop to leave out.

    This used to assert the opposite - that a usd prop is silently skipped, on the
    reasoning that its extent lives in the asset and no box should be guessed. That is
    true about guessing and wrong about skipping: the cell's part IS a usd asset, and the
    moment its declared bounds were removed it vanished from the obstacle list, leaving the
    arms planning through a part none of them could see.

    With a stage the extent is measured from the asset. Without one - as here - there is
    nothing to measure and nothing declared, and the honest answer is to say so.
    """
    manager = SimManager.get()
    previous = manager.cfg
    manager.cfg = SimConfig(props=[
        {"name": "crate", "type": "usd", "usd_path": "/x.usd", "position": [0, 0, 0]},
        {"name": "block", "type": "cube", "size": 0.1, "position": [1, 0, 0]},
    ])
    try:
        with pytest.raises(ValueError, match="would never see it"):
            manager.prop_obstacles()
    finally:
        manager.cfg = previous


def test_a_usd_prop_with_declared_bounds_is_reported():
    """A stated box is used as given, so a mock cell can still plan around an asset."""
    manager = SimManager.get()
    previous = manager.cfg
    manager.cfg = SimConfig(props=[
        {"name": "crate", "type": "usd", "usd_path": "/x.usd", "position": [0, 0, 0],
         "bounds_m": {"min": [-0.1, -0.2, 0.0], "max": [0.1, 0.2, 0.3]}},
    ])
    try:
        crate = next(o for o in manager.prop_obstacles() if o["label"] == "crate")
        assert crate["dims_mm"] == [200.0, 400.0, 300.0]
        assert crate["center_mm"] == [0.0, 0.0, 150.0]
    finally:
        manager.cfg = previous


def test_a_usd_prop_may_declare_several_boxes():
    """A conveyor is a deck, two rails and some posts - not one solid block.

    Measured as a single extent, a conveyor's box runs from the floor to the tops of its
    corner posts and from rail to rail. The part resting on the deck is inside that box
    and so is the cup at the grip pose, so the planner refuses the pick as a goal in
    collision - and refuses the place at the far end for the same reason. Declaring the
    parts separately is what leaves the space above the deck open.
    """
    manager = SimManager.get()
    previous = manager.cfg
    manager.cfg = SimConfig(props=[
        {"name": "infeed", "type": "usd", "usd_path": "/c.usd", "fixed": True,
         "position": [1.0, 0.0, 0.0], "bounds_m": [
             {"name": "deck", "min": [-1.0, -0.45, 0.71], "max": [1.0, 0.45, 0.77]},
             {"name": "rail_l", "min": [-1.0, -0.50, 0.77], "max": [1.0, -0.45, 0.79]},
             {"name": "rail_r", "min": [-1.0, 0.45, 0.77], "max": [1.0, 0.50, 0.79]},
         ]},
    ])
    try:
        boxes = {o["label"]: o for o in manager.prop_obstacles()}
        assert {"infeed.deck", "infeed.rail_l", "infeed.rail_r"} <= set(boxes)
        deck = boxes["infeed.deck"]
        assert deck["dims_mm"] == pytest.approx([2000.0, 900.0, 60.0])
        # Offset from the prop's own origin, which is where it was placed.
        assert deck["center_mm"] == pytest.approx([1000.0, 0.0, 740.0])
        assert deck["fixed"] is True
        # The space above the deck - where the part sits and the cup grips - is open.
        rail_top = boxes["infeed.rail_l"]["center_mm"][2] + \
            boxes["infeed.rail_l"]["dims_mm"][2] / 2
        assert rail_top == pytest.approx(790.0)
        assert max(b["center_mm"][2] + b["dims_mm"][2] / 2 for b in boxes.values()
                   if b["label"].startswith("infeed")) == pytest.approx(790.0)
    finally:
        manager.cfg = previous


def test_a_single_declared_box_keeps_the_bare_name():
    """One box stays labelled by the prop's own name, so nothing else has to change."""
    manager = SimManager.get()
    previous = manager.cfg
    manager.cfg = SimConfig(props=[
        {"name": "crate", "type": "usd", "usd_path": "/x.usd", "position": [0, 0, 0],
         "bounds_m": {"min": [-0.1, -0.2, 0.0], "max": [0.1, 0.2, 0.3]}},
    ])
    try:
        assert [o["label"] for o in manager.prop_obstacles()
                if o["label"] != "ground"] == ["crate"]
    finally:
        manager.cfg = previous


def test_prop_poses_reports_configured_positions_in_mock():
    """Without a stage there is nothing to read, so mock reports where props were put.

    It is flagged `live: False` so a caller can tell a spawn position from a measured
    one - the difference matters the moment anything moves.
    """
    manager = SimManager.get()
    previous_cfg, previous_mock = manager.cfg, manager.mock
    manager.cfg = SimConfig(props=[
        {"name": "part", "type": "cube", "size": 0.08, "position": [0.5, -0.4, 0.66]},
    ])
    manager.mock = True
    manager._booted.set()
    try:
        poses = manager.prop_poses()
        assert poses["part"]["position_mm"] == pytest.approx([500.0, -400.0, 660.0])
        assert poses["part"]["live"] is False
        assert manager.prop_poses(["nothing"]) == {}
    finally:
        manager.cfg, manager.mock = previous_cfg, previous_mock
        manager._booted.clear()
