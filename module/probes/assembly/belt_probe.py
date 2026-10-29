"""Trace the pallet through indexes across the belt seams, one sample every 0.1 s.

    python probes/assembly/belt_probe.py

From a restage (pallet in the tunnel): index to inside, then to the handling stop, then
run the belt on past the end of the working segment. Prints x, z and tilt of the pallet
and the shell, and what each index_to reported.
"""
from __future__ import annotations

import asyncio
import math
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from assembly.build_fragment import chosen                          # noqa: E402
from assembly.creds import options                                  # noqa: E402
from assembly.fixture import (belt_segments, index_to, restage,      # noqa: E402
                              run_belt, start_index_x)
from viam.components.generic import Generic                         # noqa: E402
from viam.robot.client import RobotClient                           # noqa: E402
from viam.services.motion import MotionClient                       # noqa: E402,F401

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")


async def main() -> int:
    robot = await RobotClient.at_address(FQDN, options())
    world = Generic.from_robot(robot, "sim-world")
    cell = chosen()
    print(f"  segments {belt_segments()}; start {start_index_x(cell)}", flush=True)
    trace = {"on": True, "t0": time.monotonic(), "last": None}

    async def sample():
        while trace["on"]:
            g = (await world.do_command({"command": "prim_poses", "paths": [
                "/World/pallet", "/World/shell"]}))["prims"]
            p, s = g["/World/pallet"], g["/World/shell"]
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(p["z_axis"][2])))))
            row = (round(p["position_mm"][0]), round(p["position_mm"][2], 1), round(tilt, 1),
                   round(s["position_mm"][0]), round(s["position_mm"][2], 1))
            if row != trace["last"]:
                print(f"    t {time.monotonic() - trace['t0']:6.2f}  pallet x {row[0]:6d} "
                      f"z {row[1]:6.1f} tilt {row[2]:4.1f}  shell x {row[3]:6d} z {row[4]:6.1f}",
                      flush=True)
                trace["last"] = row
            await asyncio.sleep(0.1)

    await restage(world, settle=2.5)
    task = asyncio.create_task(sample())
    for name in ("inside", "handle"):
        x = cell.index_x(name)
        t = time.monotonic()
        try:
            got = await index_to(world, cell, x)
            print(f"  index_to {name} ({x:.0f}): reported {got} in {time.monotonic() - t:.1f} s",
                  flush=True)
        except Exception as exc:                                     # noqa: BLE001
            print(f"  index_to {name} ({x:.0f}): FAILED {str(exc)[:200]}", flush=True)
            await run_belt(world, run=False)
            break
        await asyncio.sleep(1.0)
    print("  riding on at 420 for 4 s", flush=True)
    await run_belt(world, run=True, speed_mm_s=420.0)
    await asyncio.sleep(4.0)
    await run_belt(world, run=False)
    await asyncio.sleep(1.5)
    trace["on"] = False
    await task
    await robot.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
