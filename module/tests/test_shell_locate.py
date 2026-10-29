"""The point-cloud fit arm-c's pick aims by: PCD parsing and the pose fit, offline."""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "probes"))

from assembly import shell_locate  # noqa: E402


def _top(x, y, yaw_deg, length=452.0, width=150.0, n=4000):
    rng = np.random.default_rng(3)
    a = rng.uniform(-length / 2, length / 2, n)
    c = rng.uniform(-width / 2, width / 2, n)
    t = math.radians(yaw_deg)
    return np.stack([x + a * math.cos(t) - c * math.sin(t),
                     y + a * math.sin(t) + c * math.cos(t),
                     np.full(n, 950.0)], axis=1)


def test_fit_recovers_position_and_yaw():
    offset = 10.0
    for x, y, yaw in ((735.0, 0.0, 0.0), (760.0, -6.0, 3.0), (700.0, 4.0, -2.0)):
        t = math.radians(yaw)
        centre = (x + (offset + shell_locate.TOP_CENTRE_BIAS_MM) * math.cos(t),
                  y + (offset + shell_locate.TOP_CENTRE_BIAS_MM) * math.sin(t))
        got = shell_locate.fit(_top(*centre, yaw), offset)
        assert abs(got["x_mm"] - x) < 3.0 and abs(got["y_mm"] - y) < 3.0
        assert abs(got["yaw_deg"] - yaw) < 0.3


def test_parse_pcd_reads_xyz_and_skips_rgb():
    pts = np.zeros(3, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")])
    pts["x"], pts["y"], pts["z"], pts["rgb"] = [1, 2, 3], [4, 5, 6], [7, 8, 9], 0xFFFFFF
    head = ("VERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\n"
            "WIDTH 3\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 3\nDATA binary\n")
    got = shell_locate.parse_pcd(head.encode() + pts.tobytes())
    assert got.shape == (3, 3)
    assert np.allclose(got[:, 0], [1, 2, 3]) and np.allclose(got[:, 2], [7, 8, 9])
