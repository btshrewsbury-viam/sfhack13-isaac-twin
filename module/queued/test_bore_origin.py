"""Recovering a frame from a feature the camera saw - the one line of algebra the
vision correction stands on.

Goes to tests/ with the CV batch. Kept here until then so the live loop's offline suite
does not fail on a module that is not wired in yet.

The failure this guards against is silent and expensive: charge the bore's rotation-induced
displacement to translation and the insert aims at a point that is off by the tilt times
the bore's 400 mm lever - about 7 mm per degree - while every log line still reads like a
correct correction.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "probes"))

from assembly.asset import part                                   # noqa: E402
from assembly.build_fragment import chosen                        # noqa: E402
from assembly.film_cycle import into_frame, origin_from_bore      # noqa: E402

CELL = chosen()
PIN_INDEX_MM = 363.8
BORE = part("Shell").mm_at("assembly:pinBore:entry")


def _rot(axis: int, deg: float):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    full = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    i, j = [(1, 2), (2, 0), (0, 1)][axis]
    full[i][i], full[i][j] = c, s
    full[j][i], full[j][j] = -s, c
    return full


def _nominal():
    return (PIN_INDEX_MM, CELL.belt_y_mm, CELL.shell_origin_mm[2])


@pytest.mark.parametrize("axis,deg", [(0, 0.0), (0, 1.0), (0, 4.2), (2, 3.0), (2, -5.0)])
@pytest.mark.parametrize("shift", [(0.0, 0.0), (3.0, -2.0), (-6.0, 4.5)])
def test_the_recovered_frame_puts_the_bore_where_the_camera_saw_it(axis, deg, shift):
    nom = _nominal()
    rot = _rot(axis, deg)
    # The shell as it REALLY is: rotated, and shifted laterally (x and z - the two axes
    # across the camera's view; the bore runs along y).
    truth = [nom[0] + shift[0], nom[1], nom[2] + shift[1]]
    bore_truth = into_frame(rot, truth, BORE, point=True)

    # What the camera reports: the bore's lateral departure from where the station aimed.
    aimed = [nom[i] + BORE[i] for i in range(3)]
    seen = [bore_truth[0] - aimed[0], 0.0, bore_truth[2] - aimed[2]]

    live = origin_from_bore(rot, nom, seen, BORE)
    got = into_frame(rot, live, BORE, point=True)
    # The corrected aim lands on the bore in both measured axes.
    assert got[0] == pytest.approx(bore_truth[0], abs=1e-6)
    assert got[2] == pytest.approx(bore_truth[2], abs=1e-6)


def test_charging_the_rotation_to_translation_would_be_caught():
    """Show the test discriminates: the naive version is wrong by the lever times the tilt."""
    nom = _nominal()
    rot = _rot(0, 4.2)
    truth = list(nom)
    bore_truth = into_frame(rot, truth, BORE, point=True)
    aimed = [nom[i] + BORE[i] for i in range(3)]
    seen = [bore_truth[0] - aimed[0], 0.0, bore_truth[2] - aimed[2]]

    right = into_frame(rot, origin_from_bore(rot, nom, seen, BORE), BORE, point=True)
    naive = into_frame(rot, [nom[i] + seen[i] for i in range(3)], BORE, point=True)
    assert math.dist(right, bore_truth) < 1e-6
    # The naive answer is off by metres-per-radian on a 400 mm lever: several millimetres
    # at four degrees, which is past what the chamfer captures.
    assert math.dist(naive, bore_truth) > 1.5


def test_the_bore_is_far_enough_from_the_origin_for_this_to_matter():
    # If the bore sat on the shell's origin the correction above would be a no-op and this
    # whole file would be ceremony. It does not: state the lever.
    assert math.sqrt(sum(v * v for v in BORE)) > 200.0
