"""The vision correction's arithmetic, checked without a cell.

Everything here is the part that can be wrong silently: a sign that sends the pin further
from the hole, a singular response accepted as an answer, a Jacobian measured against one
camera mount and reused on another. Each of those produces a plausible number, which is
why they belong in a test rather than in a careful read.
"""
from __future__ import annotations

import io
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "probes"))

from assembly import bore_cv                                     # noqa: E402


def test_the_correction_moves_against_the_pixel_error():
    # A camera whose image x follows world X and whose image y follows world -Z, at
    # 2 px/mm. A hole 10 px right and 10 px down of centre is a hole that is +5 mm in X
    # and +5 mm in Z of where the station aimed, so the tool moves there.
    jac = [[2.0, 0.0], [0.0, -2.0]]
    along = bore_cv.solve(jac, (10.0, 10.0))
    assert along == pytest.approx((-5.0, 5.0))
    # And the signs flip with the error, which is the thing a transposed matrix would not do.
    assert bore_cv.solve(jac, (-10.0, -10.0)) == pytest.approx((5.0, -5.0))


def test_a_cross_coupled_camera_is_still_solved():
    # A camera rotated 45 degrees about its view axis couples both pixel axes to both world
    # axes. Back-projection through intrinsics would need to know that rotation; a measured
    # response carries it.
    r = 2.0 ** -0.5
    jac = [[2.0 * r, -2.0 * r], [2.0 * r, 2.0 * r]]
    m0, m1 = bore_cv.solve(jac, (2.0 * r * 3.0 - 2.0 * r * 4.0,
                                 2.0 * r * 3.0 + 2.0 * r * 4.0))
    assert (m0, m1) == pytest.approx((-3.0, -4.0))


def test_a_singular_response_is_refused_not_averaged():
    with pytest.raises(RuntimeError, match="singular"):
        bore_cv.solve([[1.0, 2.0], [2.0, 4.0]], (1.0, 1.0))


def test_a_jacobian_from_another_layout_is_refused(tmp_path, monkeypatch):
    monkeypatch.setattr(bore_cv, "POSES", tmp_path)
    bore_cv.save("arm-b", "pin_bore", "aaaaaaaaaaaaaaaa",
                 [[2.0, 0.0], [0.0, -2.0]], [[1, 0, 0], [0, 0, 1]])
    got, axes = bore_cv.load("arm-b", "pin_bore", "aaaaaaaaaaaaaaaa")
    assert got == [[2.0, 0.0], [0.0, -2.0]]
    assert axes == [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    with pytest.raises(bore_cv.NoJacobian, match="different camera"):
        bore_cv.load("arm-b", "pin_bore", "bbbbbbbbbbbbbbbb")


def test_a_missing_jacobian_says_so_rather_than_defaulting(tmp_path, monkeypatch):
    monkeypatch.setattr(bore_cv, "POSES", tmp_path)
    with pytest.raises(bore_cv.NoJacobian, match="never been calibrated"):
        bore_cv.load("arm-b", "pin_bore", "aaaaaaaaaaaaaaaa")


def _frame(w: int, h: int, hole=None, radius: int = 14) -> bytes:
    from PIL import Image, ImageDraw

    im = Image.new("RGB", (w, h), (190, 190, 195))
    if hole is not None:
        ImageDraw.Draw(im).ellipse(
            [hole[0] - radius, hole[1] - radius, hole[0] + radius, hole[1] + radius],
            fill=(8, 8, 10))
    buf = io.BytesIO()
    im.save(buf, format="PNG")
    return buf.getvalue()


def test_the_detector_finds_a_dark_disk_near_the_centre():
    got = bore_cv.find_hole(_frame(640, 480, (330, 250)))
    assert got is not None
    assert got[0] == pytest.approx(330.0, abs=1.5)
    assert got[1] == pytest.approx(250.0, abs=1.5)


def test_a_dark_corner_cannot_win():
    # The shell's unlit interior and the cell's dark corners are bigger and darker than the
    # bore. A detector free to pick the darkest thing anywhere would choose one of them.
    assert bore_cv.find_hole(_frame(640, 480, (40, 40), radius=60)) is None


def test_a_frame_with_no_hole_returns_nothing_rather_than_the_centre():
    assert bore_cv.find_hole(_frame(640, 480)) is None


def test_two_dark_things_do_not_average_into_a_third_place():
    """The test the first detector would have failed, and the reason it was rewritten.

    A render of the hinge has more than one dark thing in frame: the bore, and the shell's
    unlit interior behind it. Averaging every dark pixel lands the answer BETWEEN them, on
    metal - and reports a confident centroid while doing it, which the cell then acts on.
    """
    from PIL import Image, ImageDraw

    im = Image.new("RGB", (640, 480), (190, 190, 195))
    draw = ImageDraw.Draw(im)
    bore = (330, 250)
    other = (390, 300)          # still inside the search window, bigger and just as dark
    draw.ellipse([bore[0] - 12, bore[1] - 12, bore[0] + 12, bore[1] + 12], fill=(8, 8, 10))
    draw.ellipse([other[0] - 26, other[1] - 26, other[0] + 26, other[1] + 26],
                 fill=(6, 6, 8))
    buf = io.BytesIO()
    im.save(buf, format="PNG")

    got = bore_cv.find_hole(buf.getvalue())
    assert got is not None
    # The one nearest the frame centre, not the biggest and not the mean of both.
    assert got[0] == pytest.approx(bore[0], abs=2.0)
    assert got[1] == pytest.approx(bore[1], abs=2.0)
    # And it is the SMALL blob's pixel count, which is what proves it picked one region
    # rather than merging them.
    assert got[2] < 700


def test_a_blob_smaller_than_the_floor_is_not_a_bore():
    from PIL import Image, ImageDraw

    im = Image.new("RGB", (640, 480), (190, 190, 195))
    ImageDraw.Draw(im).ellipse([319, 239, 321, 241], fill=(8, 8, 10))
    buf = io.BytesIO()
    im.save(buf, format="PNG")
    assert bore_cv.find_hole(buf.getvalue()) is None
