"""The cell's taught motion, checked without a machine.

A cell like this runs one fixed sequence over and over, so it is taught once and
replayed. What has to hold is that the program on disk actually fits the round: the
round makes a numbered sequence of moves, and replay hands each one the path that was
taught for it. If the two disagree - because a move was added, removed or reordered -
replaying runs the wrong path at the wrong moment, and that is the one failure mode a
taught program has that a planned one does not.
"""

import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "probes"))

from qc.round import (  # noqa: E402
    TRAVEL_BASE_DEG, TRAVEL_PER_MM, Program, path_travel, travel_budget)

PROGRAM = Path(__file__).resolve().parents[1] / "fragments" / "qc-cell-program.json"


@pytest.fixture(scope="module")
def program():
    if not PROGRAM.exists():
        pytest.skip("the cell has not been taught yet")
    return Program.from_json(json.loads(PROGRAM.read_text()))


def test_the_program_exists_and_parses(program):
    assert program.paths, "the taught program is empty"


def test_every_step_is_a_path_of_six_axis_waypoints(program):
    for key, path in program.paths.items():
        assert path, f"{key} was taught an empty path"
        for point in path:
            assert len(point) == 6, f"{key} has a {len(point)}-axis waypoint"


def test_the_shared_steps_are_numbered_without_gaps(program):
    """Replay walks the round's moves in order and looks each one up by number.

    A gap means the teaching run made fewer moves than the round makes, so replay would
    reach a step with nothing taught for it.
    """
    shared = sorted(int(k.split(":")[0]) for k in program.paths if k[0].isdigit())
    assert shared, "no shared steps were taught"
    assert shared == list(range(min(shared), min(shared) + len(shared))), (
        f"the shared steps are not contiguous: {shared}")


def test_both_endings_were_taught(program):
    """The round is one sequence until the verdict and two after it."""
    branches = {k.split(":")[0] for k in program.paths if not k[0].isdigit()}
    assert branches == {"good", "bad"}, (
        f"the program only knows how to finish a {branches or 'neither'} round; "
        "a part sent the other way has no taught path")


def test_no_step_has_a_discontinuity_in_the_middle_of_its_path(program):
    """A dense path should not contain one enormous stride.

    Not every large step is wrong: the planner often returns a move as just its two
    endpoints, and the executor interpolates. What is wrong is a path of fifty
    waypoints with one stride many times the others in it - that is a joint wrapping
    the long way round, because pan is clamped to +-180 and two poses either side of
    the limit can only be joined the far way. It cost 304 degrees of pan twice a round
    before home stopped being planned for.
    """
    for key, path in program.paths.items():
        if len(path) < 6:
            continue          # too short to have a middle
        strides = sorted(max(abs(x - y) for x, y in zip(a, b))
                         for a, b in zip(path, path[1:]))
        median = strides[len(strides) // 2]
        assert strides[-1] < max(8.0 * median, 60.0), (
            f"{key} has a {strides[-1]:.0f} degree stride among waypoints whose "
            f"median is {median:.0f}; something wraps there")


def test_no_step_spends_more_travel_than_its_distance_justifies(program):
    """A move must cost roughly what the geometry says it costs.

    The discontinuity check above only sees a dense path, and the planner returns most
    moves as their two endpoints - so the check that matters for the majority of the
    round is this one, on the whole step rather than on one stride inside it. A step
    over budget is doing something other than the move: reaching the same place through
    a different arm configuration, or taking a quarter turn the long way because a wrist
    ran into its limit. Neither shows up as a failure - the part still arrives - which
    is why it has to be a test rather than something noticed while watching.
    """
    over = []
    for arm in ("arm-a", "arm-b"):
        for branch, sequence in runs(program, arm):
            previous = None
            for key in sequence:
                path = program.paths[key]
                travel = path_travel(path)
                target = program.targets.get(key)
                distance = (0.0 if previous is None or target is None
                            else math.dist(previous, target))
                budget = (TRAVEL_BASE_DEG if target is None
                          else travel_budget(previous, target))
                if travel > budget:
                    over.append(f"{key} ({branch}) moved {distance:.0f} mm and spent "
                                f"{travel:.0f} degrees, budget {budget:.0f}")
                if target is not None:
                    previous = target
    assert not over, "steps cost more than their geometry justifies:\n  " + \
        "\n  ".join(sorted(set(over)))


def steps_for(program, arm):
    """This arm's steps, in the order the round makes them.

    Shared steps first in numeric order, then each ending's - which is the order a round
    executes them, because the branch is only chosen once the verdict is in.
    """
    def parts(key):
        bits = key.split(":")
        return (bits[0] if len(bits) == 3 else "", int(bits[-2]), bits[-1])

    keyed = [(parts(k), k) for k in program.paths]
    mine = [(p, k) for p, k in keyed if p[2] == arm]
    shared = sorted((p, k) for p, k in mine if not p[0])
    endings = {}
    for p, k in mine:
        if p[0]:
            endings.setdefault(p[0], []).append((p, k))
    return shared, {b: sorted(v) for b, v in endings.items()}


def runs(program, arm):
    """Every whole sequence this arm executes, one per ending, as (label, keys).

    An arm that has no steps after the verdict - arm-a hands the part over and is done
    before the cell decides anything - still makes one run, and checking it was the
    point. Walking the endings alone silently skipped that arm entirely.
    """
    shared, endings = steps_for(program, arm)
    if not endings:
        return [("the round", [k for _, k in shared])]
    return [(branch, [k for _, k in shared] + [k for _, k in tail])
            for branch, tail in sorted(endings.items())]


def test_each_arms_taught_path_is_continuous(program):
    """Every step must start where that arm's previous step ended.

    A taught program is only replayable because the arm is where the next path expects
    it. Stitch two teaching runs together, or drop a step, and the arm would be
    somewhere else when the path begins - and replay does no planning, so it would
    simply drive there through whatever is in the way. This is the check that a program
    on disk still describes one continuous run.
    """
    for arm in ("arm-a", "arm-b"):
        for branch, sequence in runs(program, arm):
            for before, after in zip(sequence, sequence[1:]):
                end = program.paths[before][-1]
                start = program.paths[after][0]
                gap = max(abs(a - b) for a, b in zip(end, start))
                # Generous, because the round also issues direct joint commands - the
                # shoulder lift off the belt, the wrist unwind before a move - which run
                # live rather than being replayed and legitimately move the arm between
                # two recorded steps. What this still catches is a program stitched from
                # different teaching runs, where the gap is the width of the cell.
                assert gap < 200.0, (
                    f"{arm}'s {branch} run jumps {gap:.0f} degrees between {before} "
                    f"and {after}; the program is not one continuous run")
