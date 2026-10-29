"""The orientation maths a set-down derives its flange from.

This file exists because a hand-rolled pair of orientation-vector conversions passed
every test written against itself and was still wrong. Viam's theta is not measured from
the shortest arc that takes +z onto the vector, so the two conventions agreed for some
vectors and differed by a quarter or a half turn for others.

The lesson is in the first test: agreement with the SDK, not self-consistency, is what
makes the value mean anything - it is the motion service that has to understand it.
"""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "probes"))

from qc.round import (  # noqa: E402
    Held, _ov_to_quat, _quat_conj, _quat_mul, _quat_to_ov, _rotate_by_quat,
    flange_for, yawed,
)

# Vectors that separate the conventions: the hand-rolled version got +x right and every
# one of these wrong.
VECTORS = [(0, -1, 0, 0.0), (0, 1, 0, 0.0), (0, 1, 0, 180.0), (0, 0, -1, 0.0),
           (1, 0, 0, 90.0), (0, -1, 0, 90.0), (0, 0, 1, 45.0)]


def same_rotation(a, b, tol=1e-6):
    if sum(x * y for x, y in zip(a, b)) < 0:
        b = tuple(-v for v in b)
    return max(abs(x - y) for x, y in zip(a, b)) < tol


@pytest.mark.parametrize("vector", VECTORS)
def test_the_conversions_are_inverses(vector):
    quat = _ov_to_quat(*vector)
    back = _ov_to_quat(**_quat_to_ov(quat))
    assert same_rotation(quat, back)


def test_the_vector_is_where_the_frames_z_points():
    """The one property the whole convention rests on, checked independently."""
    for vector in VECTORS:
        quat = _ov_to_quat(*vector)
        axis = _rotate_by_quat(quat, (0.0, 0.0, 1.0))
        norm = math.sqrt(sum(v * v for v in vector[:3]))
        assert all(abs(axis[i] - vector[i] / norm) < 1e-6 for i in range(3)), vector


def test_a_yaw_is_a_yaw():
    """Yawing must turn the frame about world z and leave z alone."""
    quat = yawed(_ov_to_quat(0, -1, 0, 0.0), 180.0)
    assert same_rotation(_rotate_by_quat(quat, (0.0, 0.0, 1.0)), (0.0, 1.0, 0.0), 1e-6)


# The hold measured at the pick: the part sits 175 mm along the tool and 25 mm above the
# flange, upright, with the cup on its +y face.
PICK_FLANGE = _ov_to_quat(0.0, -1.0, 0.0, 0.0)
UPRIGHT = (1.0, 0.0, 0.0, 0.0)


def held_at_pick() -> Held:
    relative = _quat_mul(_quat_conj(PICK_FLANGE), UPRIGHT)
    offset = _rotate_by_quat(_quat_conj(PICK_FLANGE), (0.0, -175.0, 25.0))
    return Held(arm="arm-a", offset_mm=offset, dims_mm=(182.0, 110.0, 112.0),
                orientation=_quat_to_ov(relative), upright_wxyz=UPRIGHT)


def test_a_set_down_reproduces_the_pick_it_came_from():
    """Asking for the pose the part was picked from must give the pick's own flange."""
    position, orientation = flange_for(held_at_pick(), (547.0, -426.0, 664.0), UPRIGHT)
    assert all(abs(position[i] - (547.0, -251.0, 639.0)[i]) < 1e-3 for i in range(3))
    assert same_rotation(_ov_to_quat(**orientation), PICK_FLANGE, 1e-6)


def test_the_half_turn_puts_the_flange_on_the_giving_arms_side():
    """The point of the handoff.

    arm-a takes the part by its +y face because the belt blocks the other side. Set down
    yawed half a turn, that face points -y - arm-a's own side - so arm-a never reaches
    across the cell, and the free face points +y for arm-b.
    """
    position, _ = flange_for(held_at_pick(), (430.0, 0.0, 658.0),
                             yawed(UPRIGHT, 180.0))
    assert position[1] < 0.0, "arm-a would have to reach across the centreline"
    assert abs(position[1] - -175.0) < 1e-3


def test_the_part_ends_up_above_the_flange_not_below_it():
    """The bug this file was written for.

    A half turn about z leaves the part upright, so the cup is still 25 mm below its
    centre and the part still rides above the flange. The flange arithmetic that
    replaced this derivation got the sign backwards and drove the part 50 mm into the
    stand.
    """
    position, _ = flange_for(held_at_pick(), (430.0, 0.0, 658.0),
                             yawed(UPRIGHT, 180.0))
    assert position[2] == pytest.approx(633.0, abs=1e-3)


def test_an_unturned_set_down_needs_the_far_side():
    """Why the turn is not optional: without it arm-a must reach past the centreline."""
    position, _ = flange_for(held_at_pick(), (430.0, 0.0, 658.0), UPRIGHT)
    assert position[1] > 0.0
