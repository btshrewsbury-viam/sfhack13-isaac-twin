"""Does the two-check assertion actually test the handoff?

The first version did not. It compared the `shown` lists, which hold the WORLD axis each
presentation aimed at the camera - and those stop meaning anything about the part the
moment the part is turned, which is exactly what the handoff does. Both checks duly
reported the same five world axes, so the test passed on two identical sets and would
have passed just as well had the handoff never turned the part at all.

This file is the control for the replacement: synthetic traces for a cell that turns the
part and a cell that does not, asserting the tests tell them apart. It needs no machine,
so the discrimination is checked on every run rather than inferred from a green round.
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from qc.layout import Cell  # noqa: E402
from test_round_e2e import covered_by_arm_a, faces_seen  # noqa: E402

ALL_FACES = {"+x", "-x", "+y", "-y", "+z", "-z"}
UPRIGHT = [1.0, 0.0, 0.0, 0.0]


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return [w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2]


def about(axis: int, degrees: float):
    half = math.radians(degrees) / 2.0
    sin = math.sin(half)
    return [math.cos(half), sin * (axis == 0), sin * (axis == 1), sin * (axis == 2)]


class FakeTrace:
    def __init__(self, stages):
        self.stages = stages


def presentations(prefix, base, part):
    """The five poses a check runs: four rolls about the tool, then a quarter yaw."""
    stages = [
        {"stage": f"{prefix}_roll{i}", "part": list(part),
         "orientation_wxyz": qmul(about(1, roll), base)}
        for i, roll in enumerate((0, 90, 180, 270))
    ]
    stages.append({"stage": f"{prefix}_free", "part": list(part),
                   "orientation_wxyz": qmul(about(2, 90), base)})
    return stages


def round_that_turns_the_part_by(degrees: float) -> FakeTrace:
    cell = Cell.from_fragment()
    stages = [{"stage": "before_pick", "part": [547.0, -426.0, 664.0],
               "orientation_wxyz": UPRIGHT}]
    stages += presentations("first_check", UPRIGHT, cell.inspect_point_mm("arm-a"))
    stages += presentations("second_check", qmul(about(2, degrees), UPRIGHT),
                            cell.inspect_point_mm("arm-b"))
    return FakeTrace(stages)


def outcome(degrees: float):
    cell = Cell.from_fragment()
    trace = round_that_turns_the_part_by(degrees)
    covered = covered_by_arm_a(trace)
    first = faces_seen(trace, "first_check", cell.camera_mm)
    second = faces_seen(trace, "second_check", cell.camera_mm)
    return covered, first, second


def test_the_cup_lands_on_the_face_that_pointed_along_world_y():
    """The belt leaves only the +y approach, so that is the face the cup covers."""
    covered, _, _ = outcome(180.0)
    assert covered == "+y"


def test_a_turned_handoff_satisfies_every_claim():
    covered, first, second = outcome(180.0)
    assert covered not in first
    assert covered in second
    assert first | second == ALL_FACES


def test_an_unturned_handoff_fails_the_claims():
    """The control. Without this the tests above pass for the wrong reason."""
    covered, first, second = outcome(0.0)
    assert covered not in first, "the first check should never see the covered face"
    assert covered not in second, (
        "an unturned handoff leaves the covered face covered - if this passes, the "
        "test cannot tell a working handoff from a broken one")
    assert first | second != ALL_FACES


# There is deliberately no test here that the two checks' world-axis labels are
# identical. `presentations()` ignores the turn when it builds those names, so such a
# test would assert a property of this file's fake rather than of the cell. The point it
# would try to make is made by `test_an_unturned_handoff_fails_the_claims` above, which
# compares what the derivation actually returns.
