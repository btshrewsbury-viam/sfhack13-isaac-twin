"""Where a known 3D point lands in a known camera's image.

The cell has something a real one does not: it knows exactly where the flag hole is. So a
detection dataset does not need hand labelling - the boxes can be PROJECTED from the asset's
published hole position and the camera's measured pose, which makes them exact rather than
careful, and makes a thousand of them as cheap as one.

That is only worth anything if the projection is right, so `draw` exists to put the box on
the frame and let it be looked at. A projection that is quietly wrong produces a dataset
that is confidently wrong, and a detector trained on it will find the hole 40 mm to the left
for the rest of its life.

The convention is the one the module's cameras actually use: `camera_axes="usd"`, so the
camera looks down its own -Z with +X right and +Y up. Measured, not assumed - the same
question cost a round when the wrist cameras came out facing backwards.
"""
from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple


def intrinsics(fov_deg: float, width: int, height: int) -> Tuple[float, float, float, float]:
    """fx, fy, cx, cy for a pinhole with a HORIZONTAL field of view.

    Horizontal because that is what the module sets: it computes the focal length from the
    sensor's horizontal aperture, so a `fov_deg` of 69 is 69 degrees across the long axis,
    not the diagonal and not the vertical.
    """
    fx = (width / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
    return fx, fx, width / 2.0, height / 2.0


def project(point_mm: Sequence[float], cam_pos_mm: Sequence[float],
            cam_x: Sequence[float], cam_y: Sequence[float], cam_z: Sequence[float],
            fov_deg: float, width: int, height: int) -> Optional[Tuple[float, float, float]]:
    """Pixel (u, v) and range in mm, or None if the point is behind the camera."""
    fx, fy, cx, cy = intrinsics(fov_deg, width, height)
    v = [point_mm[i] - cam_pos_mm[i] for i in range(3)]
    xc = sum(v[i] * cam_x[i] for i in range(3))
    yc = sum(v[i] * cam_y[i] for i in range(3))
    zc = sum(v[i] * cam_z[i] for i in range(3))
    forward = -zc                      # usd cameras look down -Z
    if forward <= 1.0:
        return None
    return cx + fx * (xc / forward), cy - fy * (yc / forward), forward


def box_for(point_mm, radius_mm: float, **cam) -> Optional[Tuple[float, float, float, float]]:
    """A square box around a feature of known radius, in normalised image coordinates.

    The box grows and shrinks with range on its own, because it is derived from the same
    projection as the centre - which is the point of projecting rather than annotating: a
    part 40 mm nearer gets a correspondingly bigger box without anyone noticing it should.
    """
    got = project(point_mm, **cam)
    if got is None:
        return None
    u, v, forward = got
    fx, _, _, _ = intrinsics(cam["fov_deg"], cam["width"], cam["height"])
    half = fx * radius_mm / forward
    w, h = cam["width"], cam["height"]
    x0, y0 = (u - half) / w, (v - half) / h
    x1, y1 = (u + half) / w, (v + half) / h
    if x1 <= 0.0 or y1 <= 0.0 or x0 >= 1.0 or y0 >= 1.0:
        return None
    return (max(0.0, x0), max(0.0, y0), min(1.0, x1), min(1.0, y1))


def draw(frame, box, label: str = ""):
    """Put a normalised box on a PIL frame, so a projection can be LOOKED at."""
    from PIL import ImageDraw

    d = ImageDraw.Draw(frame)
    w, h = frame.size
    x0, y0, x1, y1 = box[0] * w, box[1] * h, box[2] * w, box[3] * h
    d.rectangle([x0, y0, x1, y1], outline=(0, 255, 90), width=3)
    d.line([(x0 + x1) / 2, y0 - 8, (x0 + x1) / 2, y1 + 8], fill=(0, 255, 90))
    d.line([x0 - 8, (y0 + y1) / 2, x1 + 8, (y0 + y1) / 2], fill=(0, 255, 90))
    if label:
        d.text((x0, max(0, y0 - 14)), label, fill=(0, 255, 90))
    return frame
