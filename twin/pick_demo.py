"""Camera-guided pick on sfhack13-isaac-twin.

Looks at the pick area with the wrist camera, finds objects with the pick-bar-finder and
pick-table-finder vision services (pc-crop-camera + pc-cluster on the camera's depth
cloud), grasps each with the xArm Gripper, lifts it, checks the grasp against the sim's
ground truth, and sets it down at a drop spot.

Env: VIAM_API_KEY, VIAM_API_KEY_ID, VIAM_ADDRESS (machine address). Flags:
  --look-only   detect and report, don't move to grasp
  --trials N    repeat the whole cycle N times, re-placing the props between runs
"""

import argparse
import asyncio
import json
import os

from viam.components.arm import Arm
from viam.components.generic import Generic
from viam.components.gripper import Gripper
from viam.proto.common import Pose, PoseInFrame
from viam.robot.client import RobotClient
from viam.services.motion import MotionClient
from viam.services.vision import VisionClient

# Wrist camera sits (83, -14, 18) mm from the end-effector frame. With the tool pointing
# down at theta 180 (the 850's zero-pose orientation), EE x is world +x and EE y is
# world -y, so the lens is at EE + (83, +14, -18) in world. Centre it over the pick area.
PICK_AREA_XY = (395.0, 220.0)
LOOK_HEIGHT_MM = 400.0
LOOK_POSE = Pose(x=PICK_AREA_XY[0] - 83.0, y=PICK_AREA_XY[1] - 14.0, z=LOOK_HEIGHT_MM,
                 o_x=0, o_y=0, o_z=-1, theta=180)

# Gripper frame is 105 mm down the tool axis (the real machine's frame); the finger-pad
# centre, where the sim's grip point sits, is 158 mm down, so 53 mm past the frame.
PAD_PAST_FRAME_MM = 53.0
# Fingertips reach 16.5 mm past the pad centre: keep them 1 mm off the table (z = 0).
MIN_PAD_Z_MM = 17.5
APPROACH_MM = 100.0
LIFT_MM = 150.0
DROP_XY = (520.0, 0.0)

FINDERS = {"pick-bar-finder": "pick-bar-crop", "pick-table-finder": "pick-table-crop"}
PROPS = {"pick_block": [0.395, 0.175, 0.0175], "pick_tile": [0.395, 0.262, 0.005]}


def gripper_pose(x, y, pad_z):
    return PoseInFrame(reference_frame="world",
                       pose=Pose(x=x, y=y, z=pad_z + PAD_PAST_FRAME_MM,
                                 o_x=0, o_y=0, o_z=-1, theta=180))


async def detect(robot):
    found = []
    for svc, cam in FINDERS.items():
        vision = VisionClient.from_robot(robot, svc)
        for obj in await vision.get_object_point_clouds(cam):
            for g in obj.geometries.geometries:
                c = g.center
                found.append({"finder": svc, "frame": obj.geometries.reference_frame or "world",
                              "x": c.x, "y": c.y, "z": c.z,
                              "size": [g.box.dims_mm.x, g.box.dims_mm.y, g.box.dims_mm.z]})
    return found


async def ground_truth(world):
    poses = await world.do_command({"command": "prop_poses"})
    return {k: v for k, v in poses.items() if k in PROPS}


async def pick_one(robot, motion, gripper, world, obj):
    # pc-cluster boxes are in the crop camera's frame, which is world (the crop transforms
    # the cloud to world before cutting it). The top of the box is what the camera saw.
    top_z = obj["z"] + obj["size"][2] / 2.0
    pad_z = max(top_z - min(obj["size"][2], 35.0) / 2.0, MIN_PAD_Z_MM)
    x, y = obj["x"], obj["y"]
    await gripper.open()
    await motion.move(component_name="gripper", destination=gripper_pose(x, y, pad_z + APPROACH_MM))
    await motion.move(component_name="gripper", destination=gripper_pose(x, y, pad_z))
    grabbed = await gripper.grab()
    await motion.move(component_name="gripper", destination=gripper_pose(x, y, pad_z + LIFT_MM))
    truth = await ground_truth(world)
    lifted = {k: v for k, v in truth.items()
              if (v.get("position") or v.get("z_mm") and [0, 0, v["z_mm"] / 1000])[2] * 1000 > 60}
    held = await gripper.is_holding_something()
    await motion.move(component_name="gripper",
                      destination=gripper_pose(DROP_XY[0], DROP_XY[1], MIN_PAD_Z_MM + 30))
    await gripper.open()
    await motion.move(component_name="gripper",
                      destination=gripper_pose(DROP_XY[0], DROP_XY[1], MIN_PAD_Z_MM + APPROACH_MM))
    return {"target": [round(x, 1), round(y, 1), round(pad_z, 1)], "grab": grabbed,
            "is_holding": held.is_holding_something, "lifted_props": sorted(lifted), "truth": truth}


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--look-only", action="store_true")
    ap.add_argument("--trials", type=int, default=1)
    args = ap.parse_args()
    opts = RobotClient.Options.with_api_key(api_key=os.environ["VIAM_API_KEY"],
                                            api_key_id=os.environ["VIAM_API_KEY_ID"])
    robot = await RobotClient.at_address(os.environ["VIAM_ADDRESS"], opts)
    try:
        motion = MotionClient.from_robot(robot, "builtin")
        gripper = Gripper.from_robot(robot, "gripper")
        world = Generic.from_robot(robot, "sim-world")
        report = []
        for trial in range(args.trials):
            if trial:
                for name, pos in PROPS.items():
                    await world.do_command({"command": "place_prop", "name": name, "position": pos})
            await motion.move(component_name="arm", destination=PoseInFrame(reference_frame="world", pose=LOOK_POSE))
            objs = await detect(robot)
            entry = {"trial": trial, "detected": objs, "truth": await ground_truth(world), "picks": []}
            if not args.look_only:
                for obj in objs:
                    entry["picks"].append(await pick_one(robot, motion, gripper, world, obj))
                    await motion.move(component_name="arm",
                                      destination=PoseInFrame(reference_frame="world", pose=LOOK_POSE))
            report.append(entry)
            print(json.dumps(entry, default=str))
    finally:
        await robot.close()


if __name__ == "__main__":
    asyncio.run(main())
