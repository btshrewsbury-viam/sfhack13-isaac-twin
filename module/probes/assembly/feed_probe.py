"""Watch both feeders present their parts, and print everything about them in one pass.

    python probes/assembly/feed_probe.py [--out .scratch/runs/feedprobe]

From a restage: the pin's height through every lifter step, the flag's pose from its spawn
along the strip, both against the pick they are meant to reach, with pick-cam frames at
the moments that matter.
"""
from __future__ import annotations

import argparse
import asyncio
import io
import math
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from assembly.asset import part                                     # noqa: E402
from assembly.build_fragment import chosen                          # noqa: E402
from assembly.creds import options                                  # noqa: E402
from assembly.fixture import (FLAG_FEED_SPEED_MM_S, MAG_LIFTER_PRIM,  # noqa: E402
                              feed_grasp_mm, feed_spawn, fed_upright, lower_lifter,
                              mag_attr, restage)
from PIL import Image                                               # noqa: E402
from viam.components.camera import Camera                           # noqa: E402
from viam.components.generic import Generic                         # noqa: E402
from viam.robot.client import RobotClient                           # noqa: E402
from viam.services.motion import MotionClient                       # noqa: E402,F401

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")


def _f(v):
    return "[" + ", ".join(f"{x:+.1f}" if abs(x) > 2 else f"{x:+.3f}" for x in v) + "]"


async def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=".scratch/runs/feedprobe")
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    robot = await RobotClient.at_address(FQDN, options())
    world = Generic.from_robot(robot, "sim-world")
    cam = Camera.from_robot(robot, "pick-cam")
    cell = chosen()

    async def shot(tag):
        try:
            imgs, _ = await cam.get_images()
            Image.open(io.BytesIO(imgs[0].data)).convert("RGB").save(out / f"{tag}.jpg")
        except Exception as exc:                                     # noqa: BLE001
            print(f"  shot {tag}: {exc}", flush=True)

    async def poses(paths):
        try:
            return (await world.do_command({"command": "prim_poses", "paths": paths}))["prims"]
        except Exception as exc:                                     # noqa: BLE001
            return {"error": str(exc)[-200:]}

    def feature(name, here):
        local = [v * 1000.0 for v in part(name).at("assembly:graspPosition")]
        return [here["position_mm"][i] + local[0] * here["x_axis"][i]
                + local[1] * here["y_axis"][i] + local[2] * here["z_axis"][i]
                for i in range(3)]

    def line(tag, name, here):
        g = feature(name, here)
        want = feed_grasp_mm(cell, name)
        d = [g[i] - want[i] for i in range(3)]
        print(f"  {tag:<14} {name:<4} origin {_f(here['position_mm'])} grasp-pt {_f(g)} "
              f"off {_f(d)} |{math.dist(g, want):.1f}| y_axis {_f(here['y_axis'])} "
              f"z_axis {_f(here['z_axis'])} upright={fed_upright(name, here)}", flush=True)

    for name in ("Pin", "Flag"):
        where, quat = feed_spawn(cell, name)
        print(f"  target {name}: grasp {_f(feed_grasp_mm(cell, name))} spawn {_f(where)} "
              f"quat {_f(quat)}", flush=True)
    print(f"  lifter travel {mag_attr('magazine:lifterTravel') * 1000.0:.1f} mm", flush=True)

    await lower_lifter(world)
    await restage(world, settle=2.5)
    got = await poses(["/World/pin", "/World/flag", MAG_LIFTER_PRIM])
    for k, v in got.items():
        print(f"  after restage {k}: {v if k == 'error' else _f(v['position_mm'])}",
              flush=True)
    line("restaged", "Pin", got["/World/pin"])
    line("restaged", "Flag", got["/World/flag"])
    await shot("0_restaged")

    # The lifter, one step at a time, with the pin read after each.
    travel = mag_attr("magazine:lifterTravel") * 1000.0
    for k in range(1, 11):
        await world.do_command({"command": "set_translate", "path": MAG_LIFTER_PRIM,
                                "translate_mm": [0.0, 0.0, travel * k / 10]})
        await asyncio.sleep(0.05)
        g = await poses(["/World/pin", MAG_LIFTER_PRIM])
        lz = g.get(MAG_LIFTER_PRIM, {}).get("position_mm", [0, 0, float("nan")])[2]
        print(f"  lifter {travel * k / 10:5.1f} mm: lifter prim z {lz:.1f}  "
              f"pin z {g['/World/pin']['position_mm'][2]:.2f}", flush=True)
    await asyncio.sleep(1.0)
    line("lifted+1s", "Pin", (await poses(["/World/pin"]))["/World/pin"])
    await shot("1_lifted")

    where, quat = feed_spawn(cell, "Flag")
    await world.do_command({"command": "place_prop", "name": "flag",
                            "position_mm": [float(v) for v in where],
                            "orientation_wxyz": [float(v) for v in quat]})
    line("placed", "Flag", (await poses(["/World/flag"]))["/World/flag"])
    await asyncio.sleep(0.5)
    line("placed+0.5", "Flag", (await poses(["/World/flag"]))["/World/flag"])
    await shot("2_flag_placed")
    for strip in ("flag_feed", "flag_feed_b"):
        r = await world.do_command({"command": "conveyor", "name": strip, "run": True,
                                    "speed_mm_s": FLAG_FEED_SPEED_MM_S})
        print(f"  conveyor {strip}: {r}", flush=True)
    t0 = time.monotonic()
    for t in (0.5, 1, 2, 3, 5, 8, 12, 16):
        await asyncio.sleep(max(0.0, t0 + t - time.monotonic()))
        line(f"belt {t:>4}s", "Flag", (await poses(["/World/flag"]))["/World/flag"])
        if t in (1, 3, 8, 16):
            await shot(f"3_flag_{t:02d}s")
    for strip in ("flag_feed", "flag_feed_b"):
        await world.do_command({"command": "conveyor", "name": strip, "run": False})
    await robot.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
