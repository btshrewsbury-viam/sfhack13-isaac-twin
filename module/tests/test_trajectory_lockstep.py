"""The arm must travel the segment the planner checked, not a curve of its own.

A waypoint is one position target per joint, and the joints do not arrive together: a
small correction lands at once while a gravity-loaded shoulder or a 158 degree wrist roll
is still on its way. The arm then cuts a corner through space the motion service never
cleared, touches something, and jams - reported as a stuck joint, because from the
executor's side that is all it can look like.

The mock handle has exactly this behaviour: each joint runs toward its own target at a
fixed speed, independently. That makes it the right place to test the fix, because the
property under test is about the shape of the path rather than about physics.
"""

import asyncio
import math
import sys
import threading
import time
from pathlib import Path

import pytest
from viam.components.arm import JointPositions

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from isaac_module.models.arm import IsaacArm  # noqa: E402
from isaac_module.sim_manager import MockArmHandle  # noqa: E402

START = [0.0] * 6
# One joint barely moves, one takes a half turn - the shape that fails.
GOAL = [10.0, 0.0, 0.0, 0.0, 0.0, 158.0]


class WatchedHandle(MockArmHandle):
    """A mock handle that records every target it is commanded to.

    The targets are what the test is about. The executor tracks a path by issuing a
    target a step ahead of the arm every 20 ms, so the commanded sequence IS the path;
    sampling positions instead only catches the handful of moments the executor happens
    to read them back.
    """

    SPEED = 4.0  # rad/s, so a test is quick

    def __init__(self):
        super().__init__("watched", {})
        self.seen = []

    def set_joint_targets(self, positions):
        self.seen.append(list(positions))
        return super().set_joint_targets(positions)


def arm_with(handle) -> IsaacArm:
    arm = IsaacArm("test-arm")
    arm._attrs = {}
    arm._move_timeout = 10.0
    arm._h = lambda: handle
    return arm


def worst_deviation(seen, start, goal) -> float:
    """How far the path strays from the straight segment, in degrees.

    Measured across joints at the same fraction of travel: for each sample, take the
    joint that has gone furthest as the fraction reached, and ask where the other joints
    should be at that fraction.
    """
    start = [math.radians(v) for v in start]
    goal = [math.radians(v) for v in goal]
    spans = [g - s for s, g in zip(start, goal)]
    worst = 0.0
    for sample in seen:
        fractions = [
            (c - s) / span for c, s, span in zip(sample, start, spans) if abs(span) > 1e-9
        ]
        if not fractions:
            continue
        reached = max(fractions)
        for current, base, span in zip(sample, start, spans):
            if abs(span) < 1e-9:
                continue
            worst = max(worst, abs(current - (base + span * reached)))
    return math.degrees(worst)


def run(handle, waypoints):
    arm = arm_with(handle)
    asyncio.run(arm.move_through_joint_positions(
        [JointPositions(values=w) for w in waypoints]))


def test_the_joints_stay_together_across_a_lopsided_move():
    """The property. Without lockstep the short joint finishes while the long one is
    barely started, which is a path through space nobody checked."""
    handle = WatchedHandle()
    run(handle, [GOAL])
    assert worst_deviation(handle.seen, START, GOAL) < 8.0, (
        "the arm left the straight joint-space segment the planner checked"
    )


def test_a_single_target_would_not_have_stayed_together():
    """The same move, commanded the old way, to show the test above can fail.

    Without this the test above passes for the wrong reason - an executor that quietly
    did nothing would satisfy it if the mock happened to be fast enough. Positions are
    sampled here rather than targets, because the whole point is that only one target
    is ever issued.
    """
    handle = WatchedHandle()
    handle.set_joint_targets([math.radians(v) for v in GOAL])
    sampled = []
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        sampled.append(list(handle.get_joint_positions()))
        if all(abs(c - math.radians(g)) < 1e-3 for c, g in zip(sampled[-1], GOAL)):
            break
        time.sleep(0.005)
    assert worst_deviation(sampled, START, GOAL) > 20.0


def test_the_move_still_arrives():
    handle = WatchedHandle()
    run(handle, [GOAL])
    assert all(abs(math.degrees(c) - g) < 0.5
               for c, g in zip(handle.get_joint_positions(), GOAL))


def test_every_waypoint_of_a_trajectory_is_walked():
    handle = WatchedHandle()
    run(handle, [[5.0, 0, 0, 0, 0, 40.0], [10.0, 0, 0, 0, 0, 158.0]])
    assert all(abs(math.degrees(c) - g) < 0.5
               for c, g in zip(handle.get_joint_positions(), GOAL))


def test_the_path_is_walked_rather_than_jumped_to():
    """The executor should put the arm through the path, not at the end of it.

    `_lockstep`, which used to chop a segment into five-degree sub-targets, is gone:
    the tracking executor interpolates at a fixed rate anyway, so the sub-targets were
    a second layer of the same idea. What has to survive is the property, which is that
    the arm passes through intermediate positions rather than being commanded straight
    to the goal.
    """
    handle = WatchedHandle()
    run(handle, [GOAL])
    reached = [max(abs(math.degrees(c) - g) for c, g in zip(sample, GOAL))
               for sample in handle.seen]
    assert len(reached) > 5, (
        f"the executor issued only {len(reached)} targets; it jumped to the goal "
        f"rather than walking there")
    assert max(reached) > 100.0, "the first target was already near the goal"
    assert min(reached) < 1.0, "no target ever reached the goal"
    assert reached == sorted(reached, reverse=True), (
        "the targets do not march steadily towards the goal")
