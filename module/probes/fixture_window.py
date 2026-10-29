"""Where can the fixture go, and which arm does which job?

The centre table becomes a nest that locates the mailbox. Three demands have to hold at
the same pose, and the cell's geometry has already ruled out most of the obvious answers:

  - **Only `-x` can be gripped.** It has 480 mm of clear wall and nothing proud. `+x`
    carries the flag AND both paint defects, and neither arm has IK for it anyway: both
    bases sit at x=0, so gripping `+x` means reaching past the part and pointing the tool
    back at yourself. The ends need a 229-252 mm standoff and cantilever the full 495 mm,
    which is the grip the user rejected.
  - **That fixes the part's yaw, and the yaw fixes the door.** The door is at one END of
    the part, hinged along its bottom, 0-100 degrees - it swings down and out with the
    handle at the top. With the clean face on `-x` the door necessarily faces `-y`, which
    is arm-a's side of the cell.
  - **So the two arms' jobs are not freely assignable.** The user asked for arm-a to hold
    while arm-b works the door. That needs arm-b to reach a door facing away from it,
    across the part. The alternative is to swap: arm-a seats and lets go, arm-b holds by
    the face arm-a vacated, and arm-a - whose side the door is on - opens it.

This asks the planner about both assignments at every candidate pose, rather than
choosing between them on reasoning. It screens geometry only: the part stays on the belt
and the fixture does not exist yet.
"""
import asyncio, os, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from qc.layout import Cell, CUP_APPROACH_GAP_MM
from qc.round import Clients, props_of, world_state
from viam.components.arm import Arm
from viam.components.generic import Generic
from viam.components.gripper import Gripper
from viam.proto.common import Pose, PoseInFrame
from viam.robot.client import RobotClient
from viam.services.motion import MotionClient

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
FIXTURE_X = 520.0
PART_CENTRE_ABOVE_DECK_MM = 113.5     # measured on the infeed: 769.3 deck -> 882.8 centre
DOOR_HANDLE_ABOVE_DECK_MM = 193.0     # the handle sits near the top of the door
CANDIDATES = [(deck, y) for deck in (600.0, 700.0) for y in (0.0, 200.0, 350.0, 500.0)]


async def connect() -> Clients:
    robot = await RobotClient.at_address(FQDN, RobotClient.Options.with_api_key(
        api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"]))
    return Clients(motion=MotionClient.from_robot(robot, "builtin"),
                   world=Generic.from_robot(robot, "sim-world"),
                   arms={n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b")},
                   grippers={"arm-a": Gripper.from_robot(robot, "suction-a"),
                             "arm-b": Gripper.from_robot(robot, "suction-b")}), robot


async def reach(c, arm, pose, state) -> str:
    (x, y, z), o = pose
    try:
        await c.motion.move(component_name=arm, destination=PoseInFrame(
            reference_frame="world", pose=Pose(x=x, y=y, z=z, **o)), world_state=state)
        return "ok"
    except Exception as exc:  # noqa: BLE001
        text = str(exc)
        return ("no IK" if "zero IK solutions" in text
                else "no path" if "failed to find path" in text
                else "collide" if "obstacle constraint" in text
                else "no hold" if "did not settle" in text
                else type(exc).__name__)


def poses(cell: Cell, deck: float, y: float):
    """The three poses a candidate has to support, plus the door."""
    centre = (FIXTURE_X, y, deck + PART_CENTRE_ABOVE_DECK_MM)
    half = cell.part.dims_mm[1] / 2.0
    hold = cell.side_grip(centre, "-x")
    # A cup on the door's handle end, pointing +y INTO the door, standing off by the
    # tool's length plus a working gap.
    door = ((centre[0], centre[1] - half - cell.tool_mm - CUP_APPROACH_GAP_MM,
             deck + DOOR_HANDLE_ABOVE_DECK_MM),
            {"o_x": 0.0, "o_y": 1.0, "o_z": 0.0, "theta": 0.0})
    return hold, door


async def main() -> int:
    c, robot = await connect()
    cell = Cell.from_fragment()
    print(f"  {'deck':>5} {'y':>5} | {'a holds':>8} {'b holds':>8} | "
          f"{'a door':>8} {'b door':>8} |  reach a/b at the hold")
    for deck, y in CANDIDATES:
        try:
            await c.world.do_command({"command": "reset"})
            await asyncio.sleep(2.5)
            for g in c.grippers.values():
                await g.open()
            state = world_state(await props_of(c), cell=cell, ignore=["part"])
            hold, door = poses(cell, deck, y)
            a_hold = await reach(c, "arm-a", hold, state)
            await c.world.do_command({"command": "reset"}); await asyncio.sleep(2.0)
            b_hold = await reach(c, "arm-b", hold, state)
            await c.world.do_command({"command": "reset"}); await asyncio.sleep(2.0)
            a_door = await reach(c, "arm-a", door, state)
            await c.world.do_command({"command": "reset"}); await asyncio.sleep(2.0)
            b_door = await reach(c, "arm-b", door, state)
        except Exception as exc:  # noqa: BLE001 - a dropped cell must not end the sweep
            print(f"  {deck:>5.0f} {y:>5.0f} |  cell lost: {type(exc).__name__}; "
                  f"reconnecting")
            try:
                await robot.close()
            except Exception:  # noqa: BLE001
                pass
            await asyncio.sleep(20)
            c, robot = await connect()
            continue
        ra = cell.reach_mm("arm-a", hold[0])
        rb = cell.reach_mm("arm-b", hold[0])
        # The user's assignment: arm-a holds throughout, arm-b works the door.
        asked = a_hold == "ok" and b_door == "ok"
        # The swap the geometry may force: arm-b holds by the face arm-a vacates, and
        # arm-a - whose side the door is on - opens it.
        swapped = b_hold == "ok" and a_door == "ok"
        note = ("  <== AS ASKED" if asked else
                "  <== only with the arms swapped" if swapped else "")
        print(f"  {deck:>5.0f} {y:>5.0f} | {a_hold:>8} {b_hold:>8} | "
              f"{a_door:>8} {b_door:>8} |  {ra:>4.0f}/{rb:>4.0f}{note}")
    await robot.close()
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
