"""pick_demo.py's cycle, driven through `viam machines part run` (the CLI's own login)
instead of an SDK connection, for machines this session has no API key for.

Usage: VIAM_PART_ID=<part id> python3 pick_cli.py [--trials N] [--look-only] [--randomize]
"""

import argparse
import json
import os
import random
import subprocess
import time

PART = os.environ["VIAM_PART_ID"]  # the twin's main part id

PICK_AREA_XY = (395.0, 220.0)
LOOK = {"x": PICK_AREA_XY[0] - 83.0, "y": PICK_AREA_XY[1] - 14.0, "z": 400.0,
        "o_x": 0, "o_y": 0, "o_z": -1, "theta": 180}
PAD_PAST_FRAME_MM = 53.0   # pad centre (158 mm down the tool) past the gripper frame (105 mm)
MIN_PAD_Z_MM = 17.5        # fingertips 16.5 mm past the pad centre, 1 mm off the table
APPROACH_MM = 100.0
LIFT_MM = 150.0
DROP_XY = (520.0, 0.0)
FINDERS = {"pick-bar-finder": "pick-bar-crop", "pick-table-finder": "pick-table-crop"}
PROPS = {"pick_block": [395.0, 175.0, 17.5], "pick_tile": [395.0, 258.0, 5.0]}  # mm


def call(method, data, tries=3):
    for attempt in range(tries):
        out = subprocess.run(["viam", "-q", "machines", "part", "run", "--part", PART,
                              "--method", method, "-d", json.dumps(data)],
                             capture_output=True, text=True, timeout=300)
        try:
            return json.loads(out.stdout)
        except json.JSONDecodeError:
            if attempt == tries - 1:
                raise RuntimeError(f"{method}: {out.stdout[:300]} {out.stderr[:300]}")
            time.sleep(2)


def move(component, pose):
    return call("viam.service.motion.v1.MotionService.Move",
                {"name": "builtin", "component_name": component,
                 "destination": {"reference_frame": "world", "pose": pose}})


def gripper_pose(x, y, pad_z):
    return {"x": x, "y": y, "z": pad_z + PAD_PAST_FRAME_MM, "o_x": 0, "o_y": 0, "o_z": -1, "theta": 180}


def gripper(method):
    return call(f"viam.component.gripper.v1.GripperService.{method}", {"name": "gripper"})


def world(cmd):
    return call("viam.component.generic.v1.GenericService.DoCommand",
                {"name": "sim-world", "command": cmd})["result"]


def truth():
    props = world({"command": "prop_poses"})["props"]
    return {k: [round(v, 1) for v in props[k]["position_mm"]] for k in PROPS}


def detect():
    found = []
    for svc, cam in FINDERS.items():
        d = call("viam.service.vision.v1.VisionService.GetObjectPointClouds",
                 {"name": svc, "camera_name": cam, "mime_type": "pointcloud/pcd"})
        for obj in d.get("objects", []):
            for g in obj["geometries"]["geometries"]:
                c, dims = g["center"], g["box"]["dimsMm"]
                found.append({"finder": svc, "x": c["x"], "y": c["y"], "z": c["z"],
                              "size": [dims["x"], dims["y"], dims["z"]]})
    return found


def pick(obj):
    # Boxes are in world (the crop transforms the cloud before cutting it); the box top is
    # what the camera saw. Centre the pads on the upper part of the object.
    top_z = obj["z"] + obj["size"][2] / 2.0
    pad_z = max(top_z - min(obj["size"][2], 35.0) / 2.0, MIN_PAD_Z_MM)
    x, y = obj["x"], obj["y"]
    before = truth()
    gripper("Open")
    move("gripper", gripper_pose(x, y, pad_z + APPROACH_MM))
    move("gripper", gripper_pose(x, y, pad_z))
    grabbed = gripper("Grab").get("success")
    move("gripper", gripper_pose(x, y, pad_z + LIFT_MM))
    lifted_to = truth()
    held = gripper("IsHoldingSomething").get("isHoldingSomething", False)
    lifted = [k for k in PROPS if lifted_to[k][2] - before[k][2] > 100.0]
    move("gripper", gripper_pose(DROP_XY[0], DROP_XY[1], MIN_PAD_Z_MM + 30.0))
    gripper("Open")
    move("gripper", gripper_pose(DROP_XY[0], DROP_XY[1], MIN_PAD_Z_MM + APPROACH_MM))
    near = min(before.values(), key=lambda t: (t[0] - x) ** 2 + (t[1] - y) ** 2)
    err = round(((near[0] - x) ** 2 + (near[1] - y) ** 2) ** 0.5, 1)
    return {"finder": obj["finder"], "xy_error_mm": err, "target": [round(x, 1), round(y, 1), round(pad_z, 1)],
            "grab": grabbed, "is_holding": held, "lifted": lifted,
            "after_drop": truth()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=1)
    ap.add_argument("--look-only", action="store_true")
    ap.add_argument("--randomize", action="store_true")
    args = ap.parse_args()
    ok = total = 0
    for trial in range(args.trials):
        if trial:
            for name, pos in PROPS.items():
                at, yaw = list(pos), 0.0
                if args.randomize:
                    at[0] += random.uniform(-20, 20)
                    at[1] += random.uniform(-20, 20) if name == "pick_block" else random.uniform(-10, 10)
                    yaw = random.uniform(-45, 45)
                world({"command": "place_prop", "name": name, "position_mm": at, "yaw_deg": yaw})
        move("arm", LOOK)
        objs = detect()
        entry = {"trial": trial, "truth": truth(),
                 "detected": [{k: (round(v, 1) if isinstance(v, float) else v) for k, v in o.items()}
                              for o in objs]}
        if not args.look_only:
            entry["picks"] = []
            for obj in objs:
                result = pick(obj)
                entry["picks"].append(result)
                total += 1
                ok += bool(result["lifted"]) and result["is_holding"]
                move("arm", LOOK)
        print(json.dumps(entry), flush=True)
    if total:
        print(json.dumps({"picks_ok": ok, "picks_total": total}))


if __name__ == "__main__":
    main()
