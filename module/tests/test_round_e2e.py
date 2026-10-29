"""One QC round against the running cell, asserted claim by claim.

Everything here reads the SIMULATOR, never the cell's own account of itself. The
gripper reported a grip it did not have for the better part of a day, and a test that
trusted `is_holding_something` would have passed throughout. `prop_poses` is what the
part is actually doing.

The round runs once, in a session fixture, and returns a trace. One test per claim over
a sequential round would run the round five times and fail all five identically on a bad
pick; over a trace, the failing claim names itself.
"""

import math
from pathlib import Path

import pytest

pytestmark = pytest.mark.machine

# The part settles a few millimetres into whatever it rests on, and a cup lands where
# its raycast hits rather than where arithmetic puts it. Kept below the cup's radius:
# at 30 mm a slip of a full cup-width passed.
SLOP_MM = 12.0
# Two reads half a second apart: anything still moving is not "at rest".
STILL_MM = 3.0


def stage(trace, name):
    matching = [s for s in trace.stages if s["stage"] == name]
    if not matching:
        got = ", ".join(s["stage"] for s in trace.stages) or "nothing"
        pytest.fail(f"the round never reached {name!r}; it got as far as: {got}")
    return matching[-1]


@pytest.fixture(scope="session")
def trace(good_round):
    return good_round[0]


def test_the_round_completed(good_round):
    """Reported first so a failure downstream is read as a consequence, not a cause."""
    _, failure = good_round
    assert failure is None, f"the round did not finish: {failure}"


# ---- the pick ----------------------------------------------------------------

def test_the_part_is_not_swept_off_the_belt(trace):
    """The final approach drops the part from the obstacle set, so the arm could push
    it along the belt and then close on nothing."""
    spawned = stage(trace, "before_pick")["part"]
    touched = stage(trace, "at_contact")["part"]
    assert math.dist(spawned, touched) < 10.0, (
        "the part moved before the cup closed on it")


def test_the_part_leaves_the_belt(trace, cell):
    lifted = stage(trace, "after_lift")["part"]
    assert lifted[2] > cell.belt.top_mm + 100.0, (
        f"the part is at z={lifted[2]:.0f}, not clear of the belt at "
        f"{cell.belt.top_mm:.0f}")


def test_the_part_travels_with_the_flange_through_the_first_check(trace):
    """A carry is only a carry if the part keeps its place relative to the flange.

    Measured in the FLANGE's frame. A presentation rolls the part about the tool on
    purpose, so the world-frame offset between part and flange traces a circle and is
    supposed to change - an earlier version of this test read that circle as a slip. A
    z-only check has the opposite problem: it passes while the part swings on the cup.
    """
    offsets = [
        tuple(s["offset"])
        for s in trace.stages
        if s["stage"].startswith("first_check_") and "offset" in s
    ]
    assert len(offsets) >= 2, "too few presentations to say anything about the carry"
    for other in offsets[1:]:
        assert math.dist(offsets[0], other) < SLOP_MM, (
            f"the part shifted on the cup during the first check: {offsets}")


# ---- the checks --------------------------------------------------------------

def _rotation(wxyz):
    w, x, y, z = (float(v) for v in wxyz)
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def _into_body(wxyz, vector):
    """A world vector expressed in the part's own frame: R(q) transposed times v."""
    rows = _rotation(wxyz)
    return tuple(sum(rows[i][axis] * vector[i] for i in range(3)) for axis in range(3))


def _dominant(vector) -> str:
    axis = max(range(3), key=lambda i: abs(vector[i]))
    return f"{'+' if vector[axis] > 0 else '-'}{'xyz'[axis]}"


def body_face_seen(record, camera_mm) -> str:
    """Which face of the PART the camera was looking at, from the simulator's pose.

    The trace's `{stage}_{face}` name is the WORLD axis the presentation aimed at the
    camera, which stops meaning anything about the part the moment the part is turned -
    and the handoff turns it half a turn on purpose. Both checks duly report the same
    five world axes, so a test written on those names passes whether or not the handoff
    turned anything at all.
    """
    towards = tuple(camera_mm[i] - record["part"][i] for i in range(3))
    return _dominant(_into_body(record["orientation_wxyz"], towards))


def faces_seen(trace, stage_prefix, camera_mm):
    return {
        body_face_seen(s, camera_mm)
        for s in trace.stages
        if s["stage"].startswith(stage_prefix + "_") and "orientation_wxyz" in s
        and "part" in s
    }


def covered_by_arm_a(trace) -> str:
    """The face arm-a's cup sits on, in the part's own frame.

    The belt leaves only the +y approach at the pick, so the cup lands on whichever of
    the part's faces pointed along world +y as it sat on the belt.
    """
    return _dominant(_into_body(stage(trace, "before_pick")["orientation_wxyz"],
                                (0.0, 1.0, 0.0)))


def test_the_first_check_reaches_every_pose_it_tries(trace):
    shown = stage(trace, "first_check")["shown"]
    assert set(shown) == {"+x", "-x", "+z", "-z", "-y"}, (
        f"the first check reached only {shown}")


def test_the_first_check_cannot_see_the_face_the_cup_is_on(trace, cell):
    covered = covered_by_arm_a(trace)
    seen = faces_seen(trace, "first_check", cell.camera_mm)
    assert covered not in seen, (
        f"the first check claims to have seen {covered}, which arm-a's cup is on")


def test_the_second_check_sees_the_face_the_first_could_not(trace, cell):
    """The whole reason there are two arms, and the claim the handoff exists to make.

    Read from the part's own orientation, so it fails if the handoff sets the part down
    unturned - which the world-axis labels in `shown` cannot detect.
    """
    covered = covered_by_arm_a(trace)
    seen = faces_seen(trace, "second_check", cell.camera_mm)
    assert covered in seen, (
        f"the second check never showed {covered}, the face arm-a covered; "
        f"it showed {sorted(seen)}")


def test_the_two_checks_between_them_see_every_face(trace, cell):
    seen = (faces_seen(trace, "first_check", cell.camera_mm)
            | faces_seen(trace, "second_check", cell.camera_mm))
    assert seen == {"+x", "-x", "+y", "-y", "+z", "-z"}, (
        f"the two checks saw {sorted(seen)}, missing "
        f"{sorted({'+x', '-x', '+y', '-y', '+z', '-z'} - seen)}")


# ---- the handoff -------------------------------------------------------------

def test_the_giver_lets_go(trace):
    """Asserted on the simulator: the part stays put while arm-a leaves.

    If the cup were still holding, parking arm-a would drag the part across the cell.
    """
    placed = stage(trace, "after_set_down")["part"]
    after = stage(trace, "giver_home")["part"]
    assert math.dist(placed, after) < 10.0, (
        f"the part moved {math.dist(placed, after):.0f} mm when arm-a went home; "
        "it had not let go")


def test_the_part_is_set_down_on_the_stand(trace, cell):
    placed = stage(trace, "after_set_down")["part"]
    expected = cell.resting_on(cell.fixture)
    assert math.dist(placed[:2], expected[:2]) < SLOP_MM, (
        f"the part was set down at {placed}, not on the stand at {expected}")
    assert abs(placed[2] - expected[2]) < SLOP_MM


def test_the_receiver_takes_it(trace):
    taken = stage(trace, "after_handoff")
    lifted = taken["part"]
    stood = stage(trace, "giver_home")["part"]
    assert lifted[2] > stood[2] + 50.0, (
        "the part did not come up off the stand with arm-b")


def test_each_arm_stays_on_its_own_side(trace):
    """What the half turn buys, and the reason the handoff works at all.

    Neither arm should have to reach across the centreline: arm-a's flange stays at
    negative y for the set-down, arm-b's at positive y for the pick-up.
    """
    assert stage(trace, "after_set_down")["flange"][1] < 0.0
    assert stage(trace, "after_handoff")["flange"][1] > 0.0


# ---- the place ---------------------------------------------------------------

def test_the_part_lands_in_the_tray_the_verdict_names(trace, cell):
    """Checked in z as well as xy.

    The trays are solid blocks with no walls. A part balanced on the rim, or knocked
    onto the floor beside it, is still "inside the tray" by an xy test alone - so the
    height it came to rest at is what says it is actually on the tray.
    """
    rest = stage(trace, "at_rest")
    box = cell.trays[rest["tray"]]
    part = rest["part"]
    for axis, name in ((0, "x"), (1, "y")):
        half = box.dims_mm[axis] / 2.0
        assert abs(part[axis] - box.centre_mm[axis]) <= half, (
            f"the part is outside the {rest['tray']} tray in {name}: "
            f"{part[axis]:.0f} vs {box.centre_mm[axis]:.0f} +-{half:.0f}")
    resting = cell.resting_on(box)[2]
    assert abs(part[2] - resting) < SLOP_MM, (
        f"the part rests at z={part[2]:.0f}, not on the {rest['tray']} tray at "
        f"{resting:.0f} - it is over the tray but not on it")


def test_the_part_ends_at_rest(trace):
    """The trays are solid blocks with no walls, so still-sliding is still 'inside'."""
    rest = stage(trace, "at_rest")
    moved = math.dist(rest["part"], rest["again"])
    assert moved < STILL_MM, f"the part was still moving {moved:.1f} mm later"


def test_the_part_ends_upright(trace):
    """A part knocked onto its side is inside the tray in xy and still a failure.

    Upright means the same way up it came off the belt, allowing any yaw: the round
    turns the part about z on purpose, and which way it faces in the tray is nobody's
    business.
    """
    rest = stage(trace, "at_rest")
    up = _up_axis(rest["orientation_wxyz"])
    was = _up_axis(rest["upright_wxyz"])
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, sum(a * b for a, b in zip(up, was))))))
    assert tilt < 20.0, f"the part is tilted {tilt:.0f} degrees from upright"


def _up_axis(wxyz):
    w, x, y, z = (float(v) for v in wxyz)
    return (2.0 * (x * z + y * w), 2.0 * (y * z - x * w), 1.0 - 2.0 * (x * x + y * y))


# ---- the check has to be able to change the answer ---------------------------
#
# Everything above runs on a clean part, and a cell that always said "good" would pass
# all of it. These two fixtures are the same round with one mark on the part, and the
# mark is on the face arm-a's cup covers - the one face the first check physically
# cannot see. A cell that reached its verdict from arm-a alone sends this part to the
# good tray and every other test in this file still passes.


def test_the_defective_round_completed(bad_round):
    _, failure = bad_round
    assert failure is None, f"the round did not finish: {failure}"


def test_the_mark_is_on_the_face_the_first_check_cannot_see(bad_round):
    """Guards the fixture itself.

    If the defect ever moved to a face arm-a can present, the test below would still
    pass and would no longer be testing the handoff.
    """
    trace, _ = bad_round
    showing = [name for name, on in stage(trace, "defects")["showing"].items() if on]
    assert showing == ["defect_smudge"], f"expected one mark, got {showing}"
    # Which face the mark is on comes from the cell's own derivation, so this test and
    # the round cannot disagree about it; what is checked is that it is the face arm-a
    # covers, worked out independently from the part's pose at the pick.
    faces = stage(trace, "defects")["faces"]
    assert faces["defect_smudge"] == covered_by_arm_a(trace)


def test_a_defect_only_the_second_check_can_see_still_reaches_the_bad_tray(bad_round,
                                                                           cell):
    trace, _ = bad_round
    verdict = stage(trace, "verdict")
    assert verdict["verdict"] == "bad", (
        f"the cell passed a defective part: {verdict}")
    assert verdict["found"] == ["defect_smudge"]
    rest = stage(trace, "at_rest")
    assert rest["tray"] == "bad"
    box = cell.trays["bad"]
    for axis in (0, 1):
        assert abs(rest["part"][axis] - box.centre_mm[axis]) <= box.dims_mm[axis] / 2.0


def test_a_clean_part_is_not_called_defective(trace):
    """The other half. A cell that always says "bad" passes the test above."""
    verdict = stage(trace, "verdict")
    assert verdict["verdict"] == "good", f"a clean part was rejected: {verdict}"
    assert stage(trace, "at_rest")["tray"] == "good"


def test_every_presentation_captured_a_frame(trace):
    """The frame is the check's input; a round that took none inspected nothing."""
    frames = [s.get("frame") for s in trace.stages
              if s["stage"].startswith(("first_check_", "second_check_"))
              and "showing_face" in s]
    assert frames, "no presentations recorded"
    assert all(frames), f"{frames.count(None)} presentations captured no frame"
    for path in frames:
        assert Path(path).stat().st_size > 10_000, f"{path} is suspiciously small"


def test_the_round_presents_all_six_faces_to_the_lens(trace):
    """What the oracle scores over, asserted directly.

    The verdict is deliberately the intersection of what is showing and what was
    presented, so a cell that stopped presenting a face would quietly stop finding
    defects there rather than failing.
    """
    assert set(stage(trace, "verdict")["presented"]) == {
        "+x", "-x", "+y", "-y", "+z", "-z"}


def test_the_camera_can_actually_see_the_mark(bad_round):
    """The only assertion in this suite that is about pixels.

    Everything else about a defect is bookkeeping: the oracle knows a mark is on because
    it switched it on, and knows the face was presented because it did the arithmetic
    from the simulator's poses. None of that says the mark landed inside the frustum,
    that the cup or the wrist was not in front of it, or that a 50 mm patch resolves at
    this lens and distance. The camera rendered the part as a black silhouette for the
    whole of this project's life and every one of those checks stayed green.

    Two frames of the same pose, one with the mark and one without, answer all of it.
    """
    from PIL import Image
    import numpy as np

    trace, _ = bad_round
    blinks = [s for s in trace.stages if s.get("frame_without")]
    assert blinks, (
        "no presentation ever turned a showing mark towards the lens, so nothing "
        "compared two frames - the verdict rests entirely on arithmetic")

    for shot in blinks:
        with_mark = np.asarray(Image.open(shot["frame_with"]).convert("RGB")).astype(int)
        without = np.asarray(Image.open(shot["frame_without"]).convert("RGB")).astype(int)
        assert with_mark.shape == without.shape
        changed = (np.abs(with_mark - without).max(axis=2) > 25)
        pixels = int(changed.sum())
        # A 50x32 mm patch at this lens is around 100x65 px, so a real sighting is
        # thousands of pixels. The first version of this test asked for 200 and passed
        # on 446 scattered ones, which were jpeg noise - the toggle had not reached the
        # render at all and both frames still had the mark in them.
        assert pixels > 2000, (
            f"{shot['stage']} shows {shot['marks_here']} on the {shot['showing_face']} "
            f"face, and switching it off changed only {pixels} pixels - the camera "
            f"cannot see it, whatever the geometry says")

        # And the change has to be in one place. Scattered pixels of the same count
        # would be compression noise or the arm still settling; a mark is a patch.
        ys, xs = np.nonzero(changed)
        box = (int(xs.max() - xs.min() + 1), int(ys.max() - ys.min() + 1))
        density = pixels / (box[0] * box[1])
        assert density > 0.5, (
            f"{shot['stage']}: {pixels} pixels changed but spread over a "
            f"{box[0]}x{box[1]} box ({density:.0%} filled) - that is noise, not a mark")


def test_the_mark_is_seen_on_the_face_the_first_check_cannot_show(bad_round):
    """And it is the SECOND check that sees it."""
    trace, _ = bad_round
    blinks = [s for s in trace.stages if s.get("frame_without")]
    stages = {s["stage"].split("_check_")[0] + "_check" for s in blinks}
    assert stages == {"second_check"}, (
        f"the mark was visible to {sorted(stages)}; it is supposed to be hidden from "
        "arm-a's check by arm-a's own cup")


def test_a_round_that_missed_a_face_does_not_reach_a_verdict(trace):
    """A QC cell with incomplete evidence must fault, not pass.

    When arm-b's free-face presentation failed, the round reported a defective part as
    good: the defect was listed under `missed`, the verdict came back clean, and the
    part went to the good conveyor. Nothing in the suite caught it, because every other
    assertion was about a round that had completed.
    """
    missed = [s["stage"] for s in trace.stages if "_missed_" in s["stage"]]
    assert not missed, (
        f"the round reached a verdict having skipped {missed}")
