"""What torque the shoulder needs to hold the set-down, against what it is allowed.

Measured failure: descending onto the handoff stand, arm-a stops 1.2 degrees short with
the shoulder at -144.3 wanting -145.5, creeping at 0.2 deg/s and never arriving. A
position drive has no gravity compensation - it holds a pose by deflecting, settling
where stiffness x error balances the load - so a degree of standing error means the load
needs 20000 x 0.021 = 420 N m and the drive is allowed 300. This reads the actual
efforts at the pose rather than inferring them, at the shallow solution the old program
used and the deep one the planner now picks, so the gains can be set from a number.
"""
import asyncio, math, os, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from qc.round import Clients, props_of
from viam.components.arm import Arm, JointPositions
from viam.components.generic import Generic
from viam.components.gripper import Gripper
from viam.robot.client import RobotClient
from viam.services.motion import MotionClient

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
JOINTS = ("pan", "lift", "elbow", "w1", "w2", "w3")

POSES = {
    "set-down, the shallow solution the old program used":
        [-153.5, -123.1, -100.9, -135.9, -153.5, -90.0],
    "set-down, the deep solution that will not settle":
        [-153.5, -145.5, -100.9, -135.9, -153.5, -90.0],
    "the release pose, lowest of the descent":
        [-153.5, -131.4, -102.1, -126.4, -153.5, -90.0],
    "park":
        [-153.5, -86.2, -97.4, -146.2, -153.5, -90.0],
}


async def main():
    robot = await RobotClient.at_address(FQDN, RobotClient.Options.with_api_key(
        api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"]))
    arm = Arm.from_robot(robot, "arm-a")
    world = Generic.from_robot(robot, "sim-world")
    await Gripper.from_robot(robot, "suction-a").open()
    await world.do_command({"command": "reset"})
    await asyncio.sleep(2.0)

    for name, joints in POSES.items():
        try:
            await arm.move_to_joint_positions(JointPositions(values=joints))
            arrived = True
        except Exception as exc:  # noqa: BLE001
            arrived = False
            print(f"  {name}\n    did not settle: {str(exc)[:110]}")
        await asyncio.sleep(1.5)
        got = [round(v, 1) for v in (await arm.get_joint_positions()).values]
        efforts = await arm.do_command({"command": "efforts"})
        short = max(abs(a - b) for a, b in zip(joints, got))
        print(f"  {name}")
        print(f"    wanted {joints}")
        print(f"    got    {got}   worst {short:.2f} deg short"
              f"{'' if arrived else '  (SETTLE FAILED)'}")
        for key in ("measured", "applied", "limits", "stiffness", "damping"):
            if key in efforts:
                print(f"    {key:<10} {efforts[key]}")
        print()

    await robot.close()


if __name__ == "__main__":
    asyncio.run(main())
