"""Find the hinge bore with the RealSense and say how far the insert has to move.

The pin's seated slack is 0.20 mm and its chamfer captures 1.5 mm; the carrier presents a
part to a few millimetres and the shell has 6 mm of slack in its saddle. Nothing open-loop
closes that, and three rounds of tightening the carrier's pockets each made something
worse - the last one stood the flag on its end. So the insert stops trusting geometry and
LOOKS: a frame, a hole in it, and a correction.

Two choices make this cheap and honest rather than clever.

**No intrinsics, no extrinsics.** The obvious route is to back-project the pixel error
through the camera's 69 degree field and its mount transform. That needs the focal length,
the mount's pitch, the sign of the image's up axis and the range to the target, and every
one of those is a number that can be wrong in a way that still produces a plausible
answer - which is the single bug class this cell keeps paying for. Instead the arm MOVES a
known distance along each world axis it could correct along and watches what the pixel does.
That is the Jacobian, measured, and it carries the focal length, the mount and the range
inside it without any of them being named.

**Measured once, used many times.** The probe is taught with the rest of the program and
the Jacobian is stored beside the poses under the same layout fingerprint, so a cycle pays
for one frame and one short corrective move - about 5 s on this cell - rather than for four
moves and four frames. A fingerprint mismatch refuses, exactly as a taught pose does: a
Jacobian measured against a different camera mount is a wrong number that looks right.

What this does NOT do is decide whether the shell is square. A correction aims at a target
whose axis it has not checked; `film_cycle` keeps its own tilt gate for that, and this
module deliberately does not duplicate it.
"""
from __future__ import annotations

import io
import json
import math
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

POSES = Path(__file__).resolve().parent / "poses"
# How far the arm steps while measuring the Jacobian. Big enough that the pixel shift is
# far larger than the centroid's own noise (a dark blob on a rendered frame settles to
# about a pixel), small enough that the camera stays aimed and the response stays linear:
# at the RealSense's 320 mm standoff, 8 mm is 1.4 degrees of the 69 degree field.
PROBE_STEP_MM = 8.0
# Below this the correction is not worth a planned move - the chamfer swallows it.
FLOOR_MM = 0.4
# Above this the detection is not believable. The carrier leaves a few millimetres and the
# saddle 6; 25 mm means the dark blob the detector found is not the bore, and flying a pin
# at it would be worse than inserting open-loop.
CEILING_MM = 25.0


class NoJacobian(RuntimeError):
    """There is no measured response for this layout, so nothing can be corrected."""


def _path(arm: str, task: str) -> Path:
    return POSES / arm / f"{task}.jac.json"


def save(arm: str, task: str, stamp: str, jac: Sequence[Sequence[float]],
         axes: Sequence[Sequence[float]], note: str = "") -> None:
    p = _path(arm, task)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps({
        "arm": arm, "task": task, "layout": stamp,
        # px per mm: jac[i][j] is how far pixel axis i moves per mm along world axis j.
        "jacobian_px_per_mm": [[round(float(v), 4) for v in row] for row in jac],
        "world_axes": [[round(float(v), 4) for v in a] for a in axes],
        "step_mm": PROBE_STEP_MM,
        "_note": note or ("Measured by probes/assembly/reach_check.py by stepping the arm "
                          "and watching the bore's centroid. `layout` fingerprints the "
                          "camera mount this was measured against; a different one must "
                          "refuse rather than correct by it."),
    }, indent=1) + "\n")


def load(arm: str, task: str, stamp: str) -> Tuple[List[List[float]], List[List[float]]]:
    p = _path(arm, task)
    if not p.exists():
        raise NoJacobian(
            f"{p.name} does not exist, so the camera has never been calibrated against "
            f"this station. Teach it before asking for a vision correction.")
    blob = json.loads(p.read_text())
    if blob["layout"] != stamp:
        raise NoJacobian(
            f"{p.name} was measured against layout {blob['layout']} and this cell is "
            f"{stamp}. The camera's mount is part of that fingerprint, so this response "
            f"is for a different camera and correcting by it would move the wrong way.")
    return blob["jacobian_px_per_mm"], blob["world_axes"]


# ---- seeing ----------------------------------------------------------------------

def _blobs(dark):
    """Label the connected regions of a boolean mask, with numpy and nothing else.

    The probe venv has no scipy and no OpenCV on purpose - it talks to a robot, it is not
    an image-processing environment - so this is a flood fill over the dark pixels only.
    That is cheap because a bore is a small part of the window: a few thousand pixels, not
    the ninety thousand the window holds.

    Why labelling at all, when the mean of the dark pixels is one line. Because the mean is
    not the hole. The first version averaged EVERY dark pixel in the window and its tests
    passed, which proved only that the test background was flat grey: in a real render the
    shell's unlit interior and the shadows between the hinge knuckles are also dark, and a
    mean over all of them lands between them, on metal. A centroid that is confidently in
    the wrong place is worse than no detection, because the cell acts on it.
    """
    from collections import deque

    import numpy as np

    h, w = dark.shape
    seen = np.zeros((h, w), dtype=bool)
    found = []
    ys, xs = np.nonzero(dark)
    for y0, x0 in zip(ys.tolist(), xs.tolist()):
        if seen[y0, x0]:
            continue
        queue = deque([(y0, x0)])
        seen[y0, x0] = True
        pts = []
        while queue:
            y, x = queue.popleft()
            pts.append((y, x))
            for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < h and 0 <= nx < w and dark[ny, nx] and not seen[ny, nx]:
                    seen[ny, nx] = True
                    queue.append((ny, nx))
        found.append(pts)
    return found


def find_hole(data: bytes, *, window: float = 0.3, min_px: int = 20):
    """The dark round thing NEAREST THE MIDDLE of the frame, in pixels.

    A bore is a hole: what is behind it is unlit and further away, so in a rendered frame
    it is a dark blob. That is a weaker cue than a trained detector and a far more honest
    one to teach from - it needs no dataset, it is arithmetic you can read, and when it
    fails it fails visibly rather than confidently.

    Three things keep it from being confidently wrong. The search is confined to a window
    about the frame centre, because the station aims the camera AT the bore and a detector
    free to roam picks the darkest corner of the cell. The dark pixels are grouped into
    connected regions rather than averaged, so two dark things do not average into a third
    place that is neither. And of those regions the one nearest the frame centre wins
    rather than the largest, because the shell's interior behind the hinge is both darker
    and bigger than the bore and would win on size every time.
    """
    import numpy as np
    from PIL import Image

    grey = np.asarray(Image.open(io.BytesIO(data)).convert("L"), dtype=np.float32)
    h, w = grey.shape
    hw, hh = int(w * window / 2.0), int(h * window / 2.0)
    x0, x1 = max(0, w // 2 - hw), min(w, w // 2 + hw)
    y0, y1 = max(0, h // 2 - hh), min(h, h // 2 + hh)
    win = grey[y0:y1, x0:x1]
    if win.size == 0:
        return None
    # Threshold against the window's OWN median, so the detector adapts to exposure
    # instead of carrying a brightness constant that is right for one light rig.
    dark = win < float(np.median(win)) * 0.45
    if int(dark.sum()) < min_px:
        return None
    cx, cy = (x1 - x0) / 2.0, (y1 - y0) / 2.0
    best, best_d = None, None
    for pts in _blobs(dark):
        if len(pts) < min_px:
            continue
        my = sum(p[0] for p in pts) / len(pts)
        mx = sum(p[1] for p in pts) / len(pts)
        d = (mx - cx) ** 2 + (my - cy) ** 2
        if best_d is None or d < best_d:
            best, best_d = (mx, my, len(pts)), d
    if best is None:
        return None
    return (best[0] + x0, best[1] + y0, best[2])


async def centroid(cam) -> Tuple[float, float, int, Tuple[int, int]]:
    """One frame, one detection, or a RuntimeError naming what was actually in it."""
    images, _ = await cam.get_images()
    if not images:
        raise RuntimeError("the camera returned no images, so there is nothing to find "
                           "the bore in")
    data = images[0].data
    got = find_hole(data)
    if got is None:
        from PIL import Image
        im = Image.open(io.BytesIO(data))
        raise RuntimeError(
            f"no hole found in the {im.width}x{im.height} frame - fewer than 20 pixels in "
            f"the middle 30% are dark enough to be a bore. Either the camera is not "
            f"looking at the hinge or the interior is lit as brightly as the outside.")
    from PIL import Image
    im = Image.open(io.BytesIO(data))
    return got[0], got[1], got[2], (im.width, im.height)


# ---- measuring the response -------------------------------------------------------

async def measure(cam, step, axes: Sequence[Sequence[float]]) -> List[List[float]]:
    """Step along each world axis and watch the pixel: that IS the Jacobian.

    `step` is an awaitable taking a world displacement in mm and driving the arm there and
    back is NOT its job - this walks out and back itself, so a failure part-way leaves the
    arm somewhere known.

    The step is taken BOTH ways and the response halved. A one-sided step measures the
    response plus whatever the arm's own settling contributes; a symmetric pair cancels it,
    and the difference between the two halves is a reading on whether the response is
    linear at all.
    """
    jac = [[0.0, 0.0] for _ in range(2)]
    for j, axis in enumerate(axes):
        seen = []
        for sign in (1.0, -1.0):
            await step([sign * PROBE_STEP_MM * v for v in axis])
            px, py, _, _ = await centroid(cam)
            seen.append((sign, px, py))
            await step([-sign * PROBE_STEP_MM * v for v in axis])
        (s0, x0, y0), (s1, x1, y1) = seen
        # (plus - minus) / (2 * step): the central difference.
        jac[0][j] = (x0 - x1) / (2.0 * PROBE_STEP_MM)
        jac[1][j] = (y0 - y1) / (2.0 * PROBE_STEP_MM)
    return jac


def solve(jac: Sequence[Sequence[float]], err_px: Sequence[float]) -> Tuple[float, float]:
    """How far to move along each axis to put the hole back in the middle of the frame."""
    a, b = jac[0][0], jac[0][1]
    c, d = jac[1][0], jac[1][1]
    det = a * d - b * c
    if abs(det) < 1e-9:
        raise RuntimeError(
            f"the measured pixel response is singular (jacobian {jac}), so the two axes "
            f"move the hole the same way in the image and the correction is not "
            f"determined. The camera is looking along one of them.")
    ex, ey = err_px
    # Solve J * m = -err: the move that cancels the error.
    return ((-ex * d + ey * b) / det, (-ey * a + ex * c) / det)


async def correction(cam, jac, axes) -> Tuple[List[float], float, Dict]:
    """Look once, and return the world offset the insert should be moved by.

    The returned offset is a WORLD displacement in mm, already clamped by the caller's
    judgement rather than this function's: the floor and ceiling live here as constants so
    that a caller cannot quietly choose a different pair, but the decision to refuse is the
    caller's to make and to report.
    """
    px, py, n, (w, h) = await centroid(cam)
    err = [px - w / 2.0, py - h / 2.0]
    m0, m1 = solve(jac, err)
    off = [m0 * axes[0][i] + m1 * axes[1][i] for i in range(3)]
    far = math.sqrt(sum(v * v for v in off))
    return off, far, {"pixel": [round(px, 1), round(py, 1)],
                      "centre": [w / 2.0, h / 2.0],
                      "error_px": [round(v, 1) for v in err],
                      "dark_px": n,
                      "along_axes_mm": [round(m0, 2), round(m1, 2)]}
