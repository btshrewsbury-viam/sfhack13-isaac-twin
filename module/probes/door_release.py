"""Stage the door open, let go, and watch. The check the asset was built for.

Opening the door with an arm is the hard way round and the wrong way round. Released from
open, the four conditions tell themselves apart with no arm force at all - and the arm's
remaining job, closing whatever did not close, pushes the part DOWN into its nest instead
of levering it out.

Accept: the four conditions separate. `ok` shuts, `loose` stays open, `binding` shuts
slowly, and `stuck` cannot be staged open at all - which is its reading, not an error.
"""
import asyncio, os, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from qc.layout import Cell, part_origin_offset_mm
from qc.round import Clients, part_pose
from door_check import door_angle
from viam.components.arm import Arm
from viam.components.generic import Generic
from viam.components.gripper import Gripper
from viam.robot.client import RobotClient
from viam.services.motion import MotionClient

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
CONDITIONS = ("ok", "stuck", "binding", "loose")
STAGE_DEG = 80.0


async def main() -> int:
    robot = await RobotClient.at_address(FQDN, RobotClient.Options.with_api_key(
        api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"]))
    c = Clients(motion=MotionClient.from_robot(robot, "builtin"),
                world=Generic.from_robot(robot, "sim-world"),
                arms={n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b")},
                grippers={"arm-a": Gripper.from_robot(robot, "suction-a"),
                          "arm-b": Gripper.from_robot(robot, "suction-b")})
    cell = Cell.from_fragment()
    print(f"  {'condition':10} {'at rest':>8} {'staged':>8} {'+0.5s':>8} {'+2.0s':>8} "
          f"{'+4.0s':>8}   reading")
    seen = {}
    for condition in CONDITIONS:
        # Variant BEFORE the reset: it recomposes the prim and invalidates the physics
        # views, and the reset is what rebuilds them.
        await c.world.do_command({"command": "set_variants", "prop": "part",
                                  "variants": {"door_condition": condition}})
        await c.world.do_command({"command": "reset"})
        await asyncio.sleep(3.0)
        centre = cell.resting_on(cell.fixture)
        off = part_origin_offset_mm()
        origin = [centre[i] - off[i] for i in range(3)]
        await c.world.do_command({"command": "place_prop", "name": "part",
                                  "position_mm": [origin[0], origin[1],
                                                  origin[2] + 5.0]})
        await asyncio.sleep(2.0)
        at_rest = await door_angle(c.world)
        await c.world.do_command({"command": "set_door", "degrees": STAGE_DEG})
        await asyncio.sleep(0.05)
        staged = await door_angle(c.world)
        await asyncio.sleep(0.5)
        half = await door_angle(c.world)
        await asyncio.sleep(1.5)
        two = await door_angle(c.world)
        await asyncio.sleep(2.0)
        four = await door_angle(c.world)
        # The ANGLE it settles at matters as much as whether it moved. `binding` and
        # `loose` both fail to close, and they are not the same fault: binding clamps at
        # its 35 degree limit and sits there, loose falls to 100 and stays.
        if staged < 10.0:
            reading = "will not open"          # stuck: its limit is 3 degrees
        elif four > 70.0:
            reading = "hangs open"             # loose: no spring at all
        elif four > 20.0:
            reading = "jams part-open"         # binding: stops at its limit
        else:
            reading = "closes"                 # ok: the spring takes it home
        seen[condition] = reading
        print(f"  {condition:10} {at_rest:7.1f} {staged:7.1f} {half:7.1f} {two:7.1f} "
              f"{four:7.1f}   {reading}")
    print()
    distinct = len(set(seen.values()))
    print(f"readings: {seen}")
    print(f"{distinct} distinct readings for 4 conditions - "
          f"{'PASS' if distinct == 4 else 'FAIL'}")
    await robot.close()
    return 0 if distinct == 4 else 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
