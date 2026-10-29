"""What is the shell actually resting on? Ask PhysX, at rest, before anything moves.

The shell comes to rest 24.9 degrees out of square with its origin 42 mm high, repeatably,
and three theories about why have now been wrong: a zero-separation spawn (a settling gap
changed nothing), the belt (it is already tilted before the first index), and anything
else on the pallet reaching into its seat (a test rules that out). Each wrong theory cost
a push, a restart and a teach.

So stop theorising. Fire rays at the shell from the directions a cradle could be touching
it and report what each one hits and how far away it is. Whatever is holding it up will be
the thing a ray finds where the cradle should not be.
"""
from __future__ import annotations

import asyncio
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from assembly.arms import clear_and_park                          # noqa: E402
from assembly.asset import part                                   # noqa: E402
from assembly.build_fragment import chosen                        # noqa: E402
from assembly.creds import options, FQDN                          # noqa: E402
from viam.components.arm import Arm                               # noqa: E402
from viam.components.generic import Generic                       # noqa: E402
from viam.robot.client import RobotClient                         # noqa: E402


async def main() -> int:
    robot = await RobotClient.at_address(FQDN, options())
    world = Generic.from_robot(robot, "sim-world")
    arms = {n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b", "arm-c")}
    cell = chosen()

    for which in ("flag", "door", "pin"):
        for j in (f"{which}_seat", f"{which}_clip"):
            await world.do_command({"command": "unjoin", "name": j})
    await world.do_command({"command": "reset"})
    await asyncio.sleep(4.0)
    await clear_and_park(world, arms, cell)
    await asyncio.sleep(2.0)

    got = (await world.do_command({"command": "prim_poses",
                                   "paths": ["/World/shell", "/World/pallet"]}))["prims"]
    shell, pallet = got["/World/shell"], got["/World/pallet"]
    at = shell["position_mm"]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, abs(float(shell["y_axis"][1]))))))
    want_z = cell.shell_origin_mm[2]
    print(f"  shell at {['%.1f' % v for v in at]}  tilt {tilt:.1f} deg  "
          f"(seat z {want_z:.1f}, so {at[2] - want_z:+.1f} mm high)", flush=True)
    print(f"  pallet at {['%.1f' % v for v in pallet['position_mm']]}", flush=True)
    print(f"  y_axis {['%+.3f' % v for v in shell['y_axis']]}", flush=True)

    lo, hi = part("Shell").bounds_mm
    # Fire from inside the shell's own footprint, outwards and downwards: whatever is
    # touching it is the first thing each ray meets.
    probes = []
    for x in (lo[0] + 40.0, 0.0, hi[0] - 40.0):
        for y, label in ((0.0, "centre"), (lo[1] + 6.0, "-y rail"),
                         (hi[1] - 6.0, "+y rail")):
            probes.append(((x, y, 20.0), (0.0, 0.0, -1.0), f"down at x{x:+.0f} {label}"))
    for x in (lo[0] + 40.0, hi[0] - 40.0):
        probes.append(((x, 0.0, 30.0), (0.0, -1.0, 0.0), f"out -y at x{x:+.0f}"))
        probes.append(((x, 0.0, 30.0), (0.0, 1.0, 0.0), f"out +y at x{x:+.0f}"))

    axes = [shell["x_axis"], shell["y_axis"], shell["z_axis"]]
    for local, along, label in probes:
        origin = [at[i] + sum(axes[k][i] * local[k] for k in range(3)) for i in range(3)]
        direction = [sum(axes[k][i] * along[k] for k in range(3)) for i in range(3)]
        r = (await world.do_command({"command": "raycast", "from_mm": origin,
                                     "along": direction,
                                     "distance_mm": 200.0}))["ray"]
        hit = r.get("collider") or "-"
        if r.get("hit") and "/World/shell" not in hit:
            print(f"    {label:24} -> {hit:<44} {r['distance_mm']:6.1f} mm", flush=True)
        elif r.get("hit"):
            print(f"    {label:24} -> (its own {hit.rsplit('/', 1)[-1]})", flush=True)
        else:
            print(f"    {label:24} -> nothing within 200 mm", flush=True)

    await robot.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
