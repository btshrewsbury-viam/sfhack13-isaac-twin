"""Where the reject is, from grasp-cam's point cloud - what arm-c aims its cup by.

grasp-cam looks straight down on the reject stop. Its cloud is taken into the world through
the camera's mount (measured from its prim, which is the extrinsic calibration a real cell
would have), and only the shell's upper walls and crown are kept: everything above the
door, which lies flat lower down, and inside a window round the stop. From those points:

  yaw     the long axis of their footprint (the shell is 495 x 172 mm in plan), and
  x, y    the middle of their extent along and across that axis.

The middle of the extent, not the centroid: the crown's arch puts more points on one side
of its ridge than the other wherever the camera is off its centreline, and the edges of
the footprint are where the walls actually are. The centre found this way sits a fixed
distance from the shell's origin, which is the asset's own `centre_offset_mm` along the
shell and nothing across it.

    python probes/assembly/shell_locate.py      # one reading, against the sim's truth
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path
from typing import Dict, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

CAMERA = "grasp-cam"
CAMERA_PRIM = "/World/grasp_cam"
# The band of the shell the fit uses, above the shell's origin in mm: the top of the walls
# and the crown's arch, which spans the full width. The flat door (origin -22) and the
# pallet and fixture are below it - and so is arm-c's cup, which is read ON the wall: from
# 100 its top (to +122) was in the band and put the shell 20.5 mm toward it (2026-10-14).
BAND_MM = (140.0, 240.0)
# What the band's footprint measures across on a clear view (149.6 mm, every reading);
# much wider means something else is in the cloud, and the fit is not to be trusted.
WIDTH_MM, WIDTH_SLACK_MM = 150.0, 20.0
# Half-extents round the stop, along and across the belt. ACROSS is the shell's half-width
# (86) and 64 more: at +-320 the window took in arm-c's suction unit at its stand-off, and
# the fit put the shell 72 mm toward it (2026-10-14). Its edge is at -174 there.
WINDOW_MM = (450.0, 150.0)
EXTENT_PCT = (2.0, 98.0)         # robust ends of the footprint
# The middle of what the camera sees of the top sits this far along the shell's +x from
# where `centre_offset_mm` puts the shell's middle: the band's footprint is 452 mm of the
# shell's 495, and not symmetrically. A CONSTANT, measured at four belt positions spanning
# 70 mm: -2.5, -2.4, -2.5, -2.5 mm with y -0.2 and yaw -0.04 deg every time (2026-10-14).
TOP_CENTRE_BIAS_MM = -2.45
# The camera arm-c's pick reads, set by whoever runs the cycle (film_cycle, cycle_time).
# None: measure the shell from the sim's own pose instead, as before.
CLIENT = None
# Where every cloud the pick reads is kept, with the camera's pose, for looking at later.
DUMP_DIR = Path(__file__).resolve().parents[2] / ".scratch" / "runs" / "grasp-clouds"
_dumps = 0


def parse_pcd(blob: bytes) -> np.ndarray:
    """x y z (metres, camera frame) from a binary PCD with fields x y z [rgb]."""
    head, _, body = blob.partition(b"DATA binary\n")
    fields = sizes = None
    n = 0
    for line in head.decode().splitlines():
        key, *rest = line.split()
        if key == "FIELDS":
            fields = rest
        elif key == "SIZE":
            sizes = [int(v) for v in rest]
        elif key == "POINTS":
            n = int(rest[0])
    step = sum(sizes)
    raw = np.frombuffer(body[: n * step], dtype=np.uint8).reshape(n, step)
    cols = []
    off = 0
    for f, sz in zip(fields, sizes):
        if f in ("x", "y", "z"):
            cols.append(raw[:, off:off + 4].copy().view("<f4").reshape(n))
        off += sz
    return np.stack(cols, axis=1).astype(np.float64)


async def camera_to_world(world) -> Tuple[np.ndarray, np.ndarray]:
    """Rotation and translation (mm) taking the camera's optical frame to the world.

    The optical frame is the USD camera's turned half a turn about its x: +X right, +Y
    image down, +Z along the view - the USD camera looks down its own -Z with +Y up."""
    got = (await world.do_command({"command": "prim_poses",
                                   "paths": [CAMERA_PRIM]}))["prims"][CAMERA_PRIM]
    ux, uy, uz = (np.array(got[k], dtype=float) for k in ("x_axis", "y_axis", "z_axis"))
    rot = np.stack([ux, -uy, -uz], axis=1)
    return rot, np.array(got["position_mm"], dtype=float)


def fit(points_mm: np.ndarray, centre_offset_x_mm: float) -> Dict[str, float]:
    """The shell's origin x, y and yaw from its upper points (world mm)."""
    xy = points_mm[:, :2]
    mid = xy.mean(axis=0)
    cov = np.cov((xy - mid).T)
    vals, vecs = np.linalg.eigh(cov)
    along = vecs[:, int(np.argmax(vals))]
    if along[0] < 0:                    # the shell's +x points down the line
        along = -along
    across = np.array([-along[1], along[0]])
    a = (xy - mid) @ along
    c = (xy - mid) @ across
    lo_a, hi_a = np.percentile(a, EXTENT_PCT)
    lo_c, hi_c = np.percentile(c, EXTENT_PCT)
    centre = mid + along * (lo_a + hi_a) / 2.0 + across * (lo_c + hi_c) / 2.0
    origin = centre - along * (centre_offset_x_mm + TOP_CENTRE_BIAS_MM)
    return {"x_mm": float(origin[0]), "y_mm": float(origin[1]),
            "yaw_deg": math.degrees(math.atan2(along[1], along[0])),
            "length_mm": float(hi_a - lo_a), "width_mm": float(hi_c - lo_c),
            "points": int(len(points_mm))}


async def locate(camera, world, cell, index_x: float) -> Dict[str, float]:
    """The reject's origin (x, y mm) and yaw (deg), from grasp-cam's cloud."""
    from assembly.asset import part

    blob, _mime = await camera.get_point_cloud()
    pts = parse_pcd(blob) * 1000.0
    rot, at = await camera_to_world(world)
    w = pts @ rot.T + at
    if DUMP_DIR is not None:
        DUMP_DIR.mkdir(parents=True, exist_ok=True)
        global _dumps
        _dumps += 1
        np.savez_compressed(DUMP_DIR / f"grasp-cloud-{_dumps}.npz", world_mm=w.astype("f4"),
                            rot=rot, at=at)
    z0 = cell.shell_origin_mm[2]
    keep = ((w[:, 2] > z0 + BAND_MM[0]) & (w[:, 2] < z0 + BAND_MM[1])
            & (np.abs(w[:, 0] - index_x) < WINDOW_MM[0])
            & (np.abs(w[:, 1] - cell.belt_y_mm) < WINDOW_MM[1]))
    if keep.sum() < 200:
        raise RuntimeError(f"{CAMERA}: only {int(keep.sum())} points on the shell's upper "
                           f"walls and crown - is there a shell at the stop?")
    got = fit(w[keep], part("Shell").centre_offset_mm[0])
    got["cloud_points"] = int(len(w))
    if abs(got["width_mm"] - WIDTH_MM) > WIDTH_SLACK_MM:
        raise RuntimeError(f"{CAMERA}: the shell's top reads {got['width_mm']:.0f} mm "
                           f"across against {WIDTH_MM:.0f} - something else is in the "
                           f"view, so this fit is not aimed by")
    return got


async def main() -> int:
    from viam.components.arm import Arm             # before connecting: the client only
    from viam.components.camera import Camera       # builds the types already imported
    from viam.components.generic import Generic
    from viam.robot.client import RobotClient

    from assembly.build_fragment import chosen
    from assembly.creds import options
    from assembly.fixture import index_to

    robot = await RobotClient.at_address(os.environ["QC_FQDN"], options())
    world = Generic.from_robot(robot, "sim-world")
    cell = chosen()
    from assembly.arms import clear_and_park
    arms = {n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b", "arm-c")}
    await clear_and_park(world, arms, cell)
    for dx in [float(v) for v in (sys.argv[1:] or ["0"])]:
        x = cell.index_x("handle") + dx
        await index_to(world, cell, x)
        got = await locate(Camera.from_robot(robot, CAMERA), world, cell,
                           cell.index_x("handle"))
        truth = (await world.do_command({"command": "prim_poses",
                                         "paths": ["/World/shell"]}))["prims"]["/World/shell"]
        tx, ty = truth["position_mm"][:2]
        tyaw = math.degrees(math.atan2(truth["x_axis"][1], truth["x_axis"][0]))
        print(f"  stop {dx:+.0f}: vision x {got['x_mm']:.1f} y {got['y_mm']:.1f} "
              f"yaw {got['yaw_deg']:.2f} ({got['points']} pts, {got['length_mm']:.0f} x "
              f"{got['width_mm']:.0f}); truth x {tx:.1f} y {ty:.1f} yaw {tyaw:.2f}; "
              f"error x {got['x_mm'] - tx:+.1f} y {got['y_mm'] - ty:+.1f} "
              f"yaw {got['yaw_deg'] - tyaw:+.2f}", flush=True)
    await robot.close()
    return 0


if __name__ == "__main__":
    import asyncio
    raise SystemExit(asyncio.run(main()))
