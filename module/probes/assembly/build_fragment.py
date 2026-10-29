"""Generate the Viam fragment for the assembly cell from the cell description.

ONE source, two consumers. The fable review's warning about this was specific: a harness
that re-implements placement answers reach against its own obstacle model, which is how the
previous build once reported a pick window 50 mm wide against a belt box it had invented.
So the fragment is GENERATED - never hand-edited - and `tests/test_assembly_fragment.py`
regenerates it and fails on any difference. A hand edit to the JSON is the divergence
vector, and that test is what closes it.
"""
from __future__ import annotations

import json
import os
import math
from pathlib import Path
from typing import Dict, List

from .asset import part
from .cell import Cell
from .fixture import (FLAG_FEED_SPEED_MM_S, belt_segments, start_index_x, flag_queue_mm,
                      feed_orientation, MAG_USD, _mag_origin_mm, mag_attr, mag_slots_mm,
                      feed_grasp_mm, feed_spawn, feed_travel, feeder_boxes, indexed_mm,
                      saddle)

# How much shelf to leave beyond the parts' own footprints.
SHELF_MARGIN_MM = 30.0
# Is the shell kinematic? Physics came back on in stages - the flag first, on its own,
# because the fable review asked for one insertion under physics before anything else was
# sited on a physics-off cell; then the pin and the door; and the shell last, because it is
# the one the fixture has to hold. Two 26 mm rails at +-50 mm give a 126 mm support base to
# a part 230 mm tall, and the first time this was tried the shell tipped 90 degrees the
# moment the cell booted.
#
# FALSE now, because the belt carries the fixture instead of teleporting it.
#
# It was true while indexing meant setting poses, and five rounds of that threw the
# assembly 423 m, 2.6 km, 3197 km and 13 m - moving a jointed assembly by setting poses is
# a constraint solve physx is entitled to resolve however it likes. A driven belt has none
# of that: the deck's SURFACE moves, friction carries the pallet, and the shell rides in
# the pallet's lips the way it would on a line.
#
# A kinematic shell is actively in the way of that. It is immovable, so the pallet - which
# is dynamic and sits around it - is pinned by it: the belt ran and nothing moved, and
# dropping the pallet from 150 mm up landed it on the shell at 762 rather than on the deck
# at 650. The fixture cannot hold a part that cannot be held.
# DYNAMIC, and the kinematic alternative was tried and backed out. Both have a real
# defect and this is the cheaper one.
#
# Dynamic: the shell stands on two 26 mm rails at +-50 mm, a 126 mm base under a 229 mm
# body, and it rolls at spawn - measured 24.8 degrees out of square with its origin
# 42.3 mm high, which is exactly the 100 mm rail spacing times sin 25. The cell has
# always had this; it is in this file's own history as "tipped 90 degrees off its saddle
# the moment the cell booted".
#
# Kinematic: it cannot tip, but kinematic means IMMOVABLE, so the dynamic pallet cannot
# index out from under it. Carrying it by placing it once per poll leaves it about 13 mm
# behind at 420 mm/s, and a shell out of step with the cradle it sits in is a wedge:
# measured, the pallet ran 361 mm and jammed. Dropping the cradle 3 mm to stop the wedge
# did not clear it either.
#
# So: dynamic, with a settling gap at spawn rather than a zero-separation contact, and
# the tilt measured at every index instead of assumed away. A dynamic shell is also what
# lets the belt genuinely carry the fixture, which is the thing this cell demonstrates.
SHELL_KINEMATIC = False
# Air under the shell's rails when it is first placed. A rigid body set down at exactly
# touching is a contact physx resolves on the step the pose changes, and on a base this
# narrow it resolves it by rolling. The pin needed the same courtesy for the same reason.
SHELL_SPAWN_LIFT_MM = 2.0
# The fixture rides the belt, so it cannot be kinematic - friction has nothing to push.
PALLET_KINEMATIC = False
# What the belt runs at. A fixture arrives where it arrives: this is also what sets the
# position scatter the infeed classifier's training set was jittered for.
BELT_SPEED_MM_S = 420.0
# The A06 with its downstream uprights cut at deck height (assets/conveyor, generated): they
# stood 530 mm above the belt at every segment and across the reject spur's path.
CONVEYOR_USD = str(Path(__file__).resolve().parents[2] / "assets" / "conveyor"
                   / "ConveyorBelt_A06_lowposts.usda")
# THE VISION TUNNEL (Brandon, 2026-10-12): the line's first check. An open gantry over the
# belt with three cameras - crown, +Y wall, -Y wall - and LED bars, from the usd session.
# It replaces arm-c lifting and rolling the shell under paint-cam.
GANTRY_USD = "/home/shrews/viam/usd/cell_props/vision_gantry.usda"
# Its colliders, in its own frame (m), read from the file: origin on the belt centreline at
# belt-top height, centred on the stop. Posts with their feet, the top frame, the two side
# camera drops, the top camera, and the light bars with their hangers.
GANTRY_BOUNDS_M = [
    {"name": "post_0r", "min": [-0.50, -0.80, -0.65], "max": [-0.38, -0.68, 1.10]},
    {"name": "post_0l", "min": [-0.50, 0.68, -0.65], "max": [-0.38, 0.80, 1.10]},
    {"name": "post_1r", "min": [0.60, -0.80, -0.65], "max": [0.72, -0.68, 1.10]},
    {"name": "post_1l", "min": [0.60, 0.68, -0.65], "max": [0.72, 0.80, 1.10]},
    {"name": "top", "min": [-0.46, -0.76, 1.10], "max": [0.68, 0.76, 1.18]},
    {"name": "cam_left", "min": [0.088, 0.625, 0.188], "max": [0.132, 0.738, 1.14]},
    {"name": "cam_right", "min": [0.088, -0.738, 0.188], "max": [0.132, -0.625, 1.14]},
    {"name": "cam_top", "min": [0.088, -0.022, 1.0], "max": [0.132, 0.022, 1.15]},
    {"name": "bars_top", "min": [-0.30, -0.315, 0.898], "max": [0.52, 0.315, 0.92]},
    {"name": "bar_left", "min": [-0.30, 0.585, 0.748], "max": [0.52, 0.744, 0.77]},
    {"name": "bar_right", "min": [-0.30, -0.744, 0.748], "max": [0.52, -0.585, 0.77]},
]
# The conveyor asset's boxes, in its own frame, measured from the file. The uprights sit
# at the DOWNSTREAM end and stand half a metre above the deck - a real obstacle, declared.
CONVEYOR_BOUNDS_M = [
    {"name": "deck", "min": [0.0, -0.450, 1.7415], "max": [1.9994, 0.450, 1.7805]},
    {"name": "guide_my", "min": [0.0, -0.502, 1.601], "max": [2.0, -0.452, 1.801]},
    {"name": "guide_py", "min": [0.0, 0.452, 1.601], "max": [2.0, 0.502, 1.801]},
    {"name": "post_my", "min": [1.862, -0.563, 0.0], "max": [1.937, -0.518, 1.800]},
    {"name": "post_py", "min": [1.862, 0.518, 0.0], "max": [1.937, 0.563, 1.800]},
]
# The spur runs PARALLEL to the line behind arm-c, its deck starting this far upstream of
# where the reject is set down (the shell's door then trails, -X).
REJECT_BACK_MM = 650.0
REJECT_SPEED_MM_S = 300.0
# camera -> (prim under the gantry, published horizontal fov)
# tunnel-top at 64, not the asset's 58.5: the cap sat on the frame's left border, and the
# tunnel stop now tolerates 25 mm along the belt, so the classifier would see cut-off shells
# (viam-88, 2026-10-13). 64 leaves ~8% margin each side.
TUNNEL_CAMERAS = {"tunnel-top": ("Cameras/Top", 64.0), "tunnel-left": ("Cameras/Left", 70.0),
                  "tunnel-right": ("Cameras/Right", 70.0)}
# Square columns under each arm: wide enough to look like they carry a robot, narrow enough
# not to become the thing the planner routes around.
# Slim, because the arms work BELOW their own base plane and a fat column is in the way.
#
# This did NOT fix what it was changed for, and saying so is the point of the comment.
# With the pin stations moved to the pin's real index, arm-b's reach to the tray came back
# "collision: violation between arm-b:forearm_link and pedestal_arm_b"; narrowing the
# column from 260 to 160 mm gives 50 mm a side and the collision is unchanged. So the
# constraint is the POSE, not the column: the pin's tray slot sits 66 mm below arm-b's
# base plane and 510 mm toward it in y, which folds the forearm back over its own base
# whatever is standing there.
#
# Left at 160 because it is the better number anyway - a 260 mm column under a UR5e is
# heavier than anything a UR5e needs - and because the pin station's geometry is about to
# be replaced wholesale: the asset now publishes a grasp from BELOW on a grip collar, for
# a Hand-E that is not mounted yet, and tuning a suction-cup approach to a pose that is
# being deleted would be work thrown away.
PEDESTAL_MM = 160.0
# How far past the attach point the visible fingers reach, so they close AROUND a part
# rather than in front of its near face. 18 mm takes the tips to the flag's mid-thickness
# and 18 mm along the pin's 141 mm length. Visual only - the hold is the surface gripper's
# fabricated joint, whose reach comes from the gripper's own `offset`.
GRIP_DEPTH_MM = 18.0
# The vendor assets, mirrored locally - streaming them at boot corrupts physics.
_MIRROR = "/home/shrews/viam/isaac-package-shelf/assets/mirror/"
# Schunk EZU 35: a three-jaw centric gripper, from Isaac 6.1's own asset library.
#
# Modelled Y-up in millimetres: its root carries a 90 degree X turn and a 0.001 scale, and
# referencing it under the flange replaces the root's ops - so the scale is restated here and
# the tool axis is given in that unturned frame, where it is +Y. Each jaw's translate is in
# the root's millimetres. The jaws REST with a 26 mm clear opening (inner faces at r 13,
# measured off the mesh), so closing on the pin's Ø8 collar moves each one 9 mm inward.
EZU = {
    "usd_path": _MIRROR + "Robots/Schunk/ezu_35/schunk_ezu_35.usd",
    "axis": [0.0, 1.0, 0.0],
    "mount_mm": 0.0,
    "scale": 0.001,
    "units_per_mm": 1.0,
    "rest_open_mm": 26.0,
    "fingers": {"SCHUNK_1582927_EZU_GBA_35_2": [0.0, 0.0, -1.0],
                "SCHUNK_1582927_EZU_GBA_35_3": [0.866025, 0.0, 0.5],
                "SCHUNK_1582927_EZU_GBA_35_4": [-0.866025, 0.0, 0.5]},
    "stroke_mm": 10.0, "open_mm": 40.0, "closed_mm": 0.0,
    # The usd session's pin inserts (cell_props/ezu35_pin_inserts.usda), one screwed to
    # each finger: 4 mm bars 3 mm proud of the pads, running 22 mm past the tips, so the
    # jaw grips the Ø3.4 shaft and its closed tips press the head flush (Brandon's
    # "protrusion that could push on the back of the pin", 2026-10-13).
    "insert_mm": 3.0, "min_open_mm": 8.66,
    "attachments": [
        {"usd_path": "/home/shrews/viam/usd/cell_props/ezu35_pin_inserts.usda",
         "prim": f"/EZU35PinInserts/Finger{k}", "under": finger}
        for k, finger in enumerate(("SCHUNK_1582927_EZU_GBA_35_2",
                                    "SCHUNK_1582927_EZU_GBA_35_3",
                                    "SCHUNK_1582927_EZU_GBA_35_4"))],
}
# SCHUNK EGP 40-N-N-B (0310940), the gripper Brandon has in hand (2026-10-19): a 2-finger
# servo-electric parallel gripper with Sam's slim 120 degree V fingers, on a 54.5 mm mount
# (ISO 9409-50 disc + column) so its grip centre sits 186.5 mm from the flange - the EZU's,
# so every taught approach and held offset carries over. Everything here is read off
# cell_props/egp40_gripper.usda (assets 64dcc87): +Z the approach, the jaws slide along +X;
# q = 0 is the vendor pose, FULLY CLOSED (Brandon fitted the printed fingers: his 29 / 16.5 mm
# were the jaw flanges' inside faces, so the STEP is drawn closed), the finger mount faces 28
# mm apart; 6 mm of stroke per jaw opens them to 40 mm. The pin's knob and the flag's collar
# (both Ø8) seat at q +2.62 mm, a 33.24 mm face gap. Metres, so no scale.
EGP = {
    "usd_path": "/home/shrews/viam/usd/cell_props/egp40_gripper.usda",
    "axis": [0.0, 0.0, 1.0],
    "mount_mm": 0.0,
    "units_per_mm": 0.001,
    "rest_open_mm": 28.0,
    "fingers": {"JawA": [1.0, 0.0, 0.0], "JawB": [-1.0, 0.0, 0.0]},
    "stroke_mm": 6.0, "open_mm": 40.0, "closed_mm": 33.24,
    "min_open_mm": 28.0,
}
# Overhead high-bay lights over the cell: disc lights 5 m up shining straight down, a 3 x 2
# grid over the line (x -2..2 m, y -1..1 m), warm white.
HIGH_BAYS = [{"name": f"highbay_{i}{j}", "kind": "disk", "local_position": [x, y, 5.0],
              "radius_m": 0.3, "intensity": 20000.0, "color": [1.0, 0.97, 0.92]}
             for i, x in enumerate((-2.0, 0.0, 2.0)) for j, y in enumerate((-1.0, 1.0))]
# The EGP's envelope for the planner (gripper:envelopeRadius / envelopeLength), mm.
EGP_ENVELOPE_MM = (31.5, 193.5)
# arm-a's camera: the usd session's fisheye tool (Basler ace 2 + FUJINON FE185C057HA-1),
# hung visual-only on the flange, and its lens rendered as what it is - a 185 degree
# equidistant (f-theta) fisheye, which a pinhole field cannot express.
FISHEYE_TOOL = {"usd_path": "/home/shrews/viam/usd/cell_props/fisheye_tool.usda",
                "axis": [0.0, 0.0, 1.0], "mount_mm": 0.0}
# The inside look's own light, beside the lens (Brandon, 2026-10-19): the ring flash alone
# leaves the light-grey interior dark; this lamp 13 mm ahead of the lens (out of the tool's
# housing, which shadowed it at 22 mm behind) lights it to a mid-grey mean with nothing
# clipped (measured on the dev cell).
FISHEYE_LAMP = {"name": "fisheye_lamp", "kind": "sphere", "local_position": [0.240, 0.0, 0.0],
                "radius_m": 0.012, "intensity": 90000.0, "color": [1.0, 0.98, 0.95]}
# k1..k4: the OpenCV fisheye model's terms, which the module renders the lens with - zero
# for the equidistant lens, f-theta exactly. Stated here so the page that rectifies it has a
# source for its D (viam-103 page 5, contract §1).
FISHEYE_LENS = {"fov_deg": 185.0, "projection": "equidistant",
                "k1": 0.0, "k2": 0.0, "k3": 0.0, "k4": 0.0}
# UR's short suction unit: already built along its own +X, so no rotation and no offset.
# It has no moving fingers - a cup either seals or it does not.
SUCTION = {
    "usd_path": _MIRROR + "Robots/UniversalRobots/ur10/grippers/short_gripper.usd",
    "axis": [1.0, 0.0, 0.0],
    # The model 7 mm further out than the attach point: the surface gripper will not seal
    # on a face it already touches (it needs ~6 mm), so the ray stops 8 mm off the wall
    # and the visible cup 1 mm off it (stations.CUP_GAP_MM).
    "mount_mm": 7.0,
    "fingers": {},
}
# ...measured from the PART's surface, which is `CUP_GAP_MM` further away than the tool's
# own standoff, because the same flange carries both the suction cup that does the holding
# and the fingers that show it. Those two want opposite things - the cup needs to stop
# short of the face, the fingers need to reach past it - and the only reason they are in
# tension is that this arm is wearing two grippers at once. When the Hand-E and the vacuum
# unit are split across arm-b and arm-c the coupling goes away; until then it is written
# down rather than left as two numbers that drift apart.
from .stations import THREE

FRAGMENT = Path(__file__).resolve().parents[2] / "fragments" / "assembly-cell.json"
SITING = Path(__file__).resolve().parents[2] / "fragments" / "assembly-siting.json"

ARM_ASSET = "ur5e"
# The standoffs the siting sweep settled on, in millimetres. Imported rather than restated:
# a standoff that exists twice is a standoff that will disagree with itself.
from .stations import CUP_GAP_MM, TOOLS as _TOOLS                              # noqa: E402
TOOL_STANDOFF = {k: v.standoff_mm for k, v in _TOOLS.items()}
TOOL_LENS = {k: v.lens_mm for k, v in _TOOLS.items()}
# How far to the SIDE of the tool axis each wrist camera sits, in the flange's local frame.
# arm-a carries no gripper, so its fisheye can stay on the axis - which is what the inside
# shot needs anyway, since it goes down the middle of the aperture.
# 130, not 110: the EZU's body is 195 mm across, so 110 put the camera at its rim.
CAM_SIDE_MM = {"realsense-cam": -130.0}
# realsense-cam in arm-b's END FRAME, in viam's camera convention (+Z optical axis, +X
# image right, +Y image down), so a learner's detection goes through transform_pose.
# MEASURED on the live cell, not derived: the camera prim against motion.get_pose(arm-b),
# identical at two arm poses to 0.1 mm and 0.03 deg (2026-10-13). It is the bracket
# above: 175 mm out, 130 mm aside, tilted 25.3 deg in toward the tool axis.
REALSENSE_FRAME = {"parent": "arm-b",
                   "translation": {"x": 0.0, "y": -130.0, "z": 175.0},
                   "orientation": {"type": "ov_degrees",
                                   "value": {"x": 0.0, "y": 0.4274, "z": 0.9041,
                                             "th": 180.0}}}
# Which prim each arm's tool hangs from, by the asset's own naming.
FLANGE = "/World/{prim}/wrist_3_link/flange"



def _shell_bounds(shell):
    """The shell as the planner sees it: HOLLOW, from the asset's own collider pieces - its
    floor, its two side walls, the back cap and the twelve arch pieces of the crown, with
    the front left open where the door was.

    A solid box was wrong twice over. It put the crown's full 220 mm height out at the side
    walls, where the real arch is 60 mm lower (the flag insert's ee_link refused against
    it). And it filled the interior, where arm-a's fisheye is MEANT to be, 70 mm inside the
    aperture - so once the tools went into the planner, `inside` could never have sited.
    """
    from .colliders import pieces
    got = {p["name"]: p for p in pieces("Shell")}

    def union(names, label):
        have = [got[n] for n in names if n in got]
        if not have:
            return None
        lo = [min(q["min_mm"][i] for q in have) for i in range(3)]
        hi = [max(q["max_mm"][i] for q in have) for i in range(3)]
        return {"name": label, "min": _m(lo), "max": _m(hi)}

    out = [union(["Floor", "FilletR0", "FilletR1", "FilletL0", "FilletL1"], "floor"),
           union(["WallL"], "wall_my"),
           union(["WallR_Rear", "WallR_Front", "WallR_Below", "WallR_Above"], "wall_py"),
           union(["BackCap"], "backcap"),
           union(["RailL", "RailR"], "rails")]
    for n in sorted(k for k in got if k.startswith("Arch")):
        out.append(union([n], "crown" + n[4:]))
    return [b for b in out if b is not None]



def _turned_bounds(low, high, quat):
    """A part's local box turned by `quat` (w,x,y,z) into a world-aligned box about its
    origin, mm -> m. Obstacle boxes are not rotated with a prop's orientation, so a pin
    standing collar-up would otherwise be planned around as a 167 mm bar lying across."""
    from .fixture import _quat_rotate
    pts = [_quat_rotate(quat, (x, y, z)) for x in (low[0], high[0])
           for y in (low[1], high[1]) for z in (low[2], high[2])]
    return ([min(p[i] for p in pts) for i in range(3)],
            [max(p[i] for p in pts) for i in range(3)])

def _flat_door_bounds(door):
    """The door's box as it lies FLAT, about its own origin. Obstacle boxes are not turned
    with a prop's orientation, so the closed-door bounds stood a 229 mm slab upright on
    the door shelf."""
    pos = door.at("assembly:flatPosition")
    lo = [door.at("assembly:flatEnvelopeMin")[i] - pos[i] for i in range(3)]
    hi = [door.at("assembly:flatEnvelopeMax")[i] - pos[i] for i in range(3)]
    return {"name": "door", "min": [round(v, 6) for v in lo], "max": [round(v, 6) for v in hi]}

def chosen() -> Cell:
    """The cell as the siting sweep chose it."""
    blob = json.loads(SITING.read_text())
    return Cell(belt_top_mm=blob["belt_top_mm"],
                belt_width_mm=blob["belt_width_mm"],
                belt_from_x_mm=blob["belt_from_x_mm"],
                belt_to_x_mm=blob["belt_to_x_mm"],
                arm_base_z_mm=blob["arm_base_z_mm"],
                arm_bases_mm={k: tuple(v) for k, v in blob["arm_bases_mm"].items()},
                arm_yaw_deg=dict(blob["arm_yaw_deg"]),
                rack_x_mm=blob["rack_mm"][0], rack_y_mm=blob["rack_mm"][1],
                repaint_x_mm=blob["repaint_mm"][0],
                repaint_y_mm=blob["repaint_mm"][1],
                fixture_lift_mm=blob["fixture_lift_mm"],
                index_x_for={k: float(v) for k, v in blob["belt_index_x_mm"].items()})


def _m(v) -> List[float]:
    return [round(x / 1000.0, 6) for x in v]


# EVERY FIXED CAMERA IS A THING YOU CAN SEE (Brandon, 2026-10-13): a Basler head on a ball
# mount, bolted level on a floor post - the usd session's cell_camera_head.usda and
# camera_post.usda. The head tilts in 10 degree steps and the post comes in 100 mm heights,
# so a camera is snapped onto the nearest body that exists rather than the body bent to fit.
CAMERA_HEAD_USD = "/home/shrews/viam/usd/cell_props/cell_camera_head.usda"
CAMERA_POST_USD = "/home/shrews/viam/usd/cell_props/camera_post.usda"
OVERHEAD_FIXTURE_USD = "/home/shrews/viam/usd/cell_props/overhead_camera_fixture.usda"
WAREHOUSE_USD = _MIRROR + "Environments/Simple_Warehouse/full_warehouse.usd"
# Yawed -90, the warehouse's (x, y) lands at (y, -x); its aisle centre line x = -10.5 m
# therefore lands at our y = +10.5, and this shift brings it onto the belt (y 0).
WAREHOUSE_AT_M = (0.0, -10.5)
# 3 mm up: at 0 its floor sat coplanar with the cell's matte ground plane and rendered as
# speckle (Fable review, 2026-10-14).
WAREHOUSE_LIFT_M = 0.003
# Its colliders, piece by piece (read off the asset's /Collision prims). One envelope box
# from the floor to the boom walled off everything UNDER the boom, and the planner refused
# arm-b's flag stations and arm-a's back shot against it.
OVERHEAD_FIXTURE_BOUNDS_M = [
    {"name": "base", "min": [-0.15, -0.15, 0.0], "max": [0.15, 0.15, 0.023]},
    {"name": "post", "min": [-0.04, -0.04, 0.015], "max": [0.04, 0.04, 1.982]},
    {"name": "boom", "min": [-0.02, -0.74, 1.902], "max": [0.02, 0.04, 1.986]},
    {"name": "camera", "min": [-0.022, -0.722, 1.8], "max": [0.022, -0.678, 1.902]},
]
WRIST_D4XX = {"usd_path": "/home/shrews/viam/usd/cell_props/wrist_d4xx.usda",
              "axis": [0.0, 0.0, 1.0], "mount_mm": 0.0, "prim_name": "realsense_body"}
# Read off the head's `tilt` variants (mount:tiltDeg, plateCenter, plateNormal,
# opticalHeightAbovePlate), in the head's frame: optical centre at the origin, axis +Z.
HEAD_TILTS = {
    0: ((0.0, -0.1035, -0.07415), (0.0, -1.0, 0.0), 0.1035),
    10: ((0.0, -0.102437, -0.061995), (0.0, -0.984808, 0.173648), 0.090115),
    20: ((0.0, -0.099278, -0.050209), (0.0, -0.939693, 0.34202), 0.076119),
    30: ((0.0, -0.094122, -0.03915), (0.0, -0.866025, 0.5), 0.061937),
    40: ((0.0, -0.087123, -0.029155), (0.0, -0.766044, 0.642788), 0.048),
    50: ((0.0, -0.078495, -0.020527), (0.0, -0.642788, 0.766044), 0.034731),
    60: ((0.0, -0.0685, -0.013528), (0.0, -0.5, 0.866025), 0.022534),
}
POST_HEIGHTS_M = [round(0.8 + 0.1 * k, 1) for k in range(11)]   # tube top; cap +10 mm
POST_CAP_M = 0.010


def _unit(v):
    n = math.sqrt(sum(x * x for x in v))
    return tuple(x / n for x in v)


def _quat_from_rows(m) -> List[float]:
    """wxyz from a 3x3 rotation (rows)."""
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        k = 0.5 / math.sqrt(t + 1.0)
        q = (0.25 / k, (m[2][1] - m[1][2]) * k, (m[0][2] - m[2][0]) * k,
             (m[1][0] - m[0][1]) * k)
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        k = 2.0 * math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2])
        q = ((m[2][1] - m[1][2]) / k, 0.25 * k, (m[0][1] + m[1][0]) / k,
             (m[0][2] + m[2][0]) / k)
    elif m[1][1] > m[2][2]:
        k = 2.0 * math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2])
        q = ((m[0][2] - m[2][0]) / k, (m[0][1] + m[1][0]) / k, 0.25 * k,
             (m[1][2] + m[2][1]) / k)
    else:
        k = 2.0 * math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1])
        q = ((m[1][0] - m[0][1]) / k, (m[0][2] + m[2][0]) / k, (m[1][2] + m[2][1]) / k,
             0.25 * k)
    return [round(v, 6) for v in q]


def mounted_camera(name: str, at_mm, target_mm):
    """A post and a head for a camera wanted at `at_mm` looking at `target_mm`.

    Returns (props, lens_mm, target_mm): the two props, and where the camera must actually
    stand and look so that it IS the head's lens - the tilt snapped to the head's nearest
    step, the height to the post's nearest, the look's heading kept.
    """
    d = [target_mm[i] - at_mm[i] for i in range(3)]
    reach = math.sqrt(sum(x * x for x in d))
    heading = math.atan2(d[1], d[0])
    want = math.degrees(math.atan2(-d[2], math.hypot(d[0], d[1])))
    tilt = min(HEAD_TILTS, key=lambda k: abs(k - want))
    plate, normal, above = HEAD_TILTS[tilt]
    t = math.radians(tilt)
    look = (math.cos(t) * math.cos(heading), math.cos(t) * math.sin(heading), -math.sin(t))
    # The rotation that sends the head's optical axis (+Z) along `look` and its plate's
    # outward normal straight down: the head bolted level on the post.
    def frame(a, n):
        e1 = _unit(a)
        dn = sum(n[i] * e1[i] for i in range(3))
        e2 = _unit([n[i] - dn * e1[i] for i in range(3)])
        e3 = (e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2],
              e1[0] * e2[1] - e1[1] * e2[0])
        return (e1, e2, e3)
    h, w = frame((0.0, 0.0, 1.0), normal), frame(look, (0.0, 0.0, -1.0))
    rot = [[sum(w[k][r] * h[k][c] for k in range(3)) for c in range(3)] for r in range(3)]
    tube = min(POST_HEIGHTS_M, key=lambda z: abs(z + POST_CAP_M + above - at_mm[2] / 1000.0))
    lens_z = (tube + POST_CAP_M + above) * 1000.0
    # Then slide the post along its heading until the snapped ray passes through the
    # target, so the target is mid-frame however narrow the lens.
    if tilt > 0:
        out = (lens_z - target_mm[2]) / math.tan(t)
        lens = (target_mm[0] - out * math.cos(heading), target_mm[1] - out * math.sin(heading),
                lens_z)
        reach = math.dist(lens, target_mm)
    else:
        lens = (at_mm[0], at_mm[1], lens_z)
    off = [sum(rot[r][c] * plate[c] for c in range(3)) * 1000.0 for r in range(3)]
    base = (lens[0] + off[0], lens[1] + off[1], 0.0)
    top_m = tube + POST_CAP_M
    props = [
        {"name": f"{name.replace('-', '_')}_post", "type": "usd", "usd_path": CAMERA_POST_USD,
         "prim": "/CameraPost", "fixed": True, "articulation_root": False,
         "position": _m(base), "yaw_deg": 0.0,
         "variants": {"height": f"h{int(round(tube * 1000)):04d}"},
         # The post and the head on it, for the planner.
         "bounds_m": [{"name": "base", "min": [-0.15, -0.15, 0.0], "max": [0.15, 0.15, 0.025]},
                      {"name": "post", "min": [-0.045, -0.045, 0.0],
                       "max": [0.045, 0.045, round(top_m, 3)]},
                      {"name": "head", "min": [-0.13, -0.13, round(top_m, 3)],
                       "max": [0.13, 0.13, round(top_m + 0.16, 3)]}]},
        {"name": f"{name.replace('-', '_')}_head", "type": "usd", "usd_path": CAMERA_HEAD_USD,
         "prim": "/CameraHead", "fixed": True, "articulation_root": False,
         "position": _m(lens), "orientation_wxyz": _quat_from_rows(rot),
         "variants": {"tilt": f"down{tilt:02d}"}},
    ]
    aim = tuple(lens[i] + look[i] * reach for i in range(3))
    return props, lens, aim


def _cube(box, colour, fixed=True) -> Dict:
    return {"name": box.name, "type": "cube", "fixed": fixed, "size": 1.0,
            "scale": _m(box.dims_mm), "position": _m(box.centre_mm), "color": colour}


# The trained infeed classifier, as the registry holds it.
#
# Named here rather than looked up at push time because a deployment that silently follows
# "whatever trained last" is a deployment nobody can reproduce. Bump the version when a new
# model is trained and the change is in the diff.
INFEED_MODEL = {"name": "mailbox-infeed-v3",
                "org": "cd5185ed-7553-464b-84aa-ebf099ecc838",
                "version": "20261007-043024"}


def _vision() -> Dict[str, List[Dict]]:
    """The ml model and the vision service that wraps it, plus what they need to run.

    Three pieces, and leaving any of them out fails in a different place: the PACKAGE
    fetches the trained model, the MODULE supplies the tflite runtime that executes it, and
    the vision service is what turns an ml model's output tensors into classifications the
    rest of the cell can ask for by name.
    """
    ref = f"{INFEED_MODEL['org']}/{INFEED_MODEL['name']}"
    pkg = "${packages.ml_model." + INFEED_MODEL["name"] + "}"
    return {
        "packages": [{"name": INFEED_MODEL["name"], "package": ref,
                      "version": INFEED_MODEL["version"], "type": "ml_model"}],
        # `viam:tflite_cpu`, which is what the registry actually calls it - the module-data
        # directory on this machine is `viam_tflite_cpu`, from an earlier machine that used
        # it. A module id that does not resolve does not fail cleanly; it resolved to
        # something else and ran the wrong first_run.
        "modules": [{"type": "registry", "name": "viam_tflite_cpu",
                     "module_id": "viam:tflite_cpu", "version": "latest"}],
        "services": [
            {"name": "infeed-ml", "type": "mlmodel", "namespace": "rdk",
             # `viam:mlmodel-tflite:tflite_cpu`, from the module's own meta.json. The
             # module is `viam:tflite_cpu` and the model it registers is not named after
             # it, which is worth writing down because guessing produced "unknown resource
             # type ... not registered" with the module sitting there loaded.
             "model": "viam:mlmodel-tflite:tflite_cpu",
             "attributes": {"model_path": f"{pkg}/{INFEED_MODEL['name']}.tflite",
                            "label_path": f"{pkg}/labels.txt", "num_threads": 2}},
            # The vision service expects a `probability` tensor for a classifier; the
            # training script's output carries it, so no remapping is needed here. If a
            # future model is trained elsewhere, that is the first thing to check - a name
            # mismatch does not fail, it guesses.
            {"name": "infeed-vision", "type": "vision", "namespace": "rdk",
             "model": "mlmodel", "attributes": {"mlmodel_name": "infeed-ml"}},
        ],
    }


FILM_POST_Y_MM = 1100.0
# Which way grasp-cam's boom reaches, from its post to the lens over the handling stop: a
# 700 mm boom, so the post stands at (+420, +560) from the lens - off the belt on the +Y
# side and 605 mm from arm-b's base.
GRASP_BOOM_DIR = (-0.6, -0.8)
# A camera looking straight down, as a Viam frame (+Z optical, +X image right, +Y image
# down). Without one the frame system took the optical axis for world +Z and any
# transform_pose from it came out flipped.
#
# th 180, re-MEASURED for grasp-cam from its rendered prim (2026-10-14): image right is
# world +X, along the belt, and image down world -Y. Viam builds an ov as
# Rz(lon) Ry(lat) Rz(th), so with the axis at -Z the frame's +X is (-cos th, sin th, 0):
# the -90 measured on 2026-10-13 (for the overhead camera too, gone since) now put image
# right at world -Y, 90 degrees from what the camera renders - every transform_pose from
# grasp-cam turned a quarter. The look-at straight down has no natural "up", so this is a
# fact of the current placement, checked by .scratch/vmtools/cam_frames.py.
DOWN_CAMERA_OV = {"type": "ov_degrees", "value": {"x": 0.0, "y": 0.0, "z": -1.0, "th": 180.0}}


def detail_cameras(cell: Cell):
    """(name, position_mm, target_mm) for each close camera the film insets."""
    shell = part("Shell")
    _, oy, oz = cell.shell_origin_mm
    entry = shell.mm_at("assembly:pinBore:entry")
    hole = shell.mm_at("assembly:flagHole:entry")
    bore = (cell.index_x("pin") + entry[0], oy + entry[1], oz + entry[2])
    flag = (cell.index_x("flag") + hole[0], oy + hole[1], oz + hole[2])
    pin_at, flag_at = feed_grasp_mm(cell, "Pin"), feed_grasp_mm(cell, "Flag")
    feeders = tuple((pin_at[i] + flag_at[i]) / 2.0 for i in range(3))
    # ON POSTS, behind arm-b on the +Y side where the pin and the flag go in, out of its
    # working zone - a camera has to stand on something (Brandon, 2026-10-13: every camera
    # modelled and visible). Long lenses, since they now stand a metre off.
    return [
        # (No pick camera: on a post behind arm-b it filmed arm-b's elbow, and the picks
        # read in the overview.)
        # Off arm-b's shoulder on the +Y side (hinge) and downstream (flag hole), looking
        # past the jaw's side: from straight behind arm-b, arm-b filled the frame and the
        # pin was never seen; the hinge camera downstream was hidden by the flag camera's
        # post (2026-10-13). Clear of grasp-cam's post at (1155, 560).
        # Far out on the +Y side, 1.4 m off on a long lens: 400 mm from arm-b's base its
        # head was in arm-b's forearm path at the flag-hole look (reach, 2026-10-13). The
        # line of sight clears arm-b's base by 300 mm and the other posts by 270.
        ("hinge-cam", (1000.0, 900.0, bore[2] + 200.0),
         (bore[0], bore[1] + 20.0, bore[2]), 14.0),
        ("flag-cam", (flag[0] + 780.0, 650.0, flag[2] + 300.0),
         (flag[0], flag[1] + 10.0, flag[2]), 14.0),
    ]


def build(cell: Cell) -> Dict:
    shell = part("Shell")
    props: List[Dict] = []

    # The conveyor, then the saddle, then the shell IN the saddle - all at one index.
    #
    # The saddle and the shell have to be placed at the SAME belt position, which is
    # obvious and was wrong in the first version: the saddle went to the `inside` index at
    # x -55 and the shell to `work_x_mm` at 600, so the shell was standing over bare belt
    # 655 mm away from its own fixture. With physics on it simply fell the 100 mm the
    # saddle would have lifted it, which is how the mistake announced itself.
    start = start_index_x(cell)
    # The belt is DRIVEN: a flat deck with a surface velocity, which is how this module
    # moves a belt and the only way a part on a fixture can be indexed without being
    # teleported. Five rounds of teleporting a jointed assembly threw it 423 m, 2.6 km,
    # 3197 km and 13 m; a conveyor just carries it.
    # A REAL conveyor, not a painted box. The cube deck was a stand-in, and it could be any
    # length because a cube can be - which is how the cell ended up with 4.6 m of belt for
    # stations spanning 950 mm. This asset is the one the two-arm cell ran on, so its driven
    # scope and its mode are known rather than guessed.
    #
    # Where the asset goes, read off the asset rather than guessed. Opened from disk, the
    # file puts its origin at the UPSTREAM END of the deck and runs the deck along its own
    # local +X: the belt collider measures x 0.0..1.9994, y -0.450..+0.450,
    # z 1.7415..1.7805,
    # the side guides y 0.452..0.502 standing to z 1.801, and the two uprights at
    # x 1.862..1.937 at the far end. So:
    #
    #   * NO YAW. The asset already runs along +X, which is the way this cell indexes. The
    #     previous -90 was written on the belief that the asset ran along its own -Y; it
    #     does not, and the rotation put 2 m of belt across the cell with the pallet on the
    #     floor beside it. The rotation was applied correctly the whole time - the measured
    #     x 1050..1950 / y -2000..0 is exactly what -90 about x 1500 produces - so the bug
    #     was the number, not the authoring, and three rounds were spent looking at the
    #     authoring.
    #   * ORIGIN AT THE UPSTREAM END, so the deck covers belt_from_x..belt_to_x, which the
    #     siting file already sizes to the asset's 1999 mm.
    #   * z AT belt_top - 1780.5, the deck collider's own top, so a part set down at
    #     belt_top rests on the deck rather than 0.5 mm inside it.
    for seg_name, dx in belt_segments():
        props.append({
            "name": seg_name, "type": "usd", "usd_path": CONVEYOR_USD,
            "position": _m((cell.belt_from_x_mm + dx, cell.belt_y_mm,
                            cell.belt_top_mm - 1780.5)),
            "yaw_deg": 0.0, "fixed": True, "articulation_root": False, "friction": 0.9,
            "conveyor": {"driven": "Belt", "mode": "belt", "axis": [1.0, 0.0, 0.0],
                         "speed_mm_s": BELT_SPEED_MM_S},
            # The boxes the asset actually has, in its own frame, measured from the file. The
            # uprights sit at the DOWNSTREAM end and stand half a metre above the deck, which
            # is a real obstacle and is declared as one rather than left out because it is
            # inconvenient. At belt_from_x they land past the last station.
            "bounds_m": CONVEYOR_BOUNDS_M,
        })
    # THE REJECT SPUR, parallel to the line on arm-c's far side. arm-c sets a failed shell
    # on it with one 180 degree base swing - the spur is sited at the grasp mirrored through
    # arm-c's base (siting `repaint_mm`) - and it carries the shell away downstream.
    props.append({
        "name": "reject_belt", "type": "usd", "usd_path": CONVEYOR_USD,
        "position": _m((cell.repaint_x_mm - REJECT_BACK_MM, cell.repaint_y_mm,
                        cell.belt_top_mm - 1780.5)),
        "yaw_deg": 0.0, "fixed": True, "articulation_root": False, "friction": 0.9,
        "conveyor": {"driven": "Belt", "mode": "belt", "axis": [1.0, 0.0, 0.0],
                     "speed_mm_s": REJECT_SPEED_MM_S},
        "bounds_m": CONVEYOR_BOUNDS_M,
    })
    if "tunnel" in cell.index_x_for:
        props.append({
            "name": "vision_gantry", "type": "usd", "usd_path": GANTRY_USD,
            "prim": "/VisionGantry", "fixed": True, "articulation_root": False,
            "position": _m((cell.index_x("tunnel"), cell.belt_y_mm, cell.belt_top_mm)),
            "yaw_deg": 0.0, "bounds_m": GANTRY_BOUNDS_M,
        })
    # THE WAREHOUSE AROUND IT (Brandon, 2026-10-13): Isaac's Simple_Warehouse, turned so its
    # central aisle (11 m wide, clear for 50 m between the rack blocks) runs along the belt
    # with the cell in the middle of it. Static scenery: no planner bounds, nothing near it
    # moves. The cell's own floor stays for physics and goes matte, so this one is seen.
    props.append({
        "name": "warehouse", "type": "usd", "usd_path": WAREHOUSE_USD, "prim": "/Root",
        "fixed": True, "articulation_root": False,
        "position": [WAREHOUSE_AT_M[0], WAREHOUSE_AT_M[1], WAREHOUSE_LIFT_M], "yaw_deg": -90.0,
        # Not an obstacle (measured, it would be one building-sized box with every arm
        # inside it); its own floor collider and sun lights off - this cell has both.
        "obstacle": False,
        # Its roof off as well: a closed building shuts out the dome, and with its own 37
        # lamps on the deck washed out white. Lamps off, roof open: lit as the cell was.
        "deactivate": ["GroundPlane"], "lights_off": True,
        "deactivate_prefix": ["SM_CeilingA"],
    })
    # PEDESTALS. The arms have been standing on nothing at 800 mm, which looks like what it
    # is: a column from the floor to each arm's base.
    for arm_name, base in sorted(cell.arm_bases_mm.items()):
        props.append({
            "name": f"pedestal_{arm_name.replace('-', '_')}", "type": "cube",
            "fixed": True, "size": 1.0,
            "scale": _m((PEDESTAL_MM, PEDESTAL_MM, base[2])),
            "position": _m((base[0], base[1], base[2] / 2.0)),
            "color": [0.30, 0.31, 0.34],
        })
    # And the fixture is ONE composite body, not four boxes that happen to be adjacent.
    #
    # Four separate dynamic props would be four parts that start next to each other and
    # slide apart on the first step. A composite prop is what this module already has for
    # "several pieces of geometry that have to move as one thing": the parent carries the
    # rigid body and the children carry only geometry and collision.
    pieces = saddle((start, cell.belt_y_mm, 0.0), cell.belt_top_mm, cell.fixture_lift_mm)
    anchor = (start, cell.belt_y_mm, cell.belt_top_mm)
    props.append({
        "name": "pallet", "type": "cube", "fixed": PALLET_KINEMATIC,
        # 40 kg, and the mass is doing a job. A pallet is held on a belt by friction alone:
        # mu * m * g, which at 6 kg and a 0.9 deck is 53 N - less than an arm pressing a pin
        # 135 mm through a bore, and measured, the whole fixture slid 110 mm off the
        # centreline the first time a full cycle ran on the belt. At 40 kg it is 353 N,
        # which holds, and is why real pallets are heavy rather than light. A surface
        # velocity drives kinematically, so the extra mass costs nothing at indexing.
        "position": _m(anchor), "mass_kg": 40.0, "color": [0.45, 0.47, 0.5],
        "friction": 0.9,
        "children": [{"name": b.name, "shape": "cube",
                      "position": _m((b.centre_mm[i] - anchor[i] for i in range(3))),
                      "scale": _m(b.dims_mm)} for b in pieces],
    })

    origin = (start, cell.belt_y_mm,
              cell.shell_origin_mm[2] + (0.0 if SHELL_KINEMATIC
                                         else SHELL_SPAWN_LIFT_MM))
    props.append({
        "name": "shell", "type": "usd", "usd_path": shell.usd_path,
        # KINEMATIC while the layout is being established, which is what "physics off
        # first" means in this module: `fixed` makes a prop kinematic rather than disabled,
        # so it still collides and still reports where it is, but it does not fall.
        #
        # Needed immediately. With physics on the shell tipped 90 degrees off its saddle
        # the moment the cell booted, because two 26 mm rails at +-50 mm give a 126 mm
        # support base to a part 230 mm tall - and `reset` restages props 5 mm up and lets
        # them drop, which is enough. That is a real finding about the saddle and it is
        # filed for when physics comes back on; it is not a reason to stop establishing
        # where the stations are.
        "fixed": SHELL_KINEMATIC,
        "position": _m(origin), "yaw_deg": 0.0, "scale": 1.0,
        "articulation_root": False, "body_prim": shell.prim,
        "bounds_m": _shell_bounds(shell),
        "variants": {"scheme": "classic_black", "paint_defect": "none",
                     "interior_defect": "none"},
    })

    components: List[Dict] = [{
        "name": "sim-world", "namespace": "rdk", "type": "generic",
        "model": "viam-education:isaac-sim:world",
        "attributes": {
            # No livestream: it streams the kit viewport, and that viewport rendered every
            # frame with nobody connected - 20 of an idle frame's 28 ms (2026-10-14, L4).
            # Learners watch through the Viam cameras; the film grabs those too.
            "headless": True, "livestream": False,
            # Measured (2026-10-14, L4): physics is 2 ms a step and a rendered frame 22-31,
            # so rendering every 1/60 s step held the sim to 0.53 of real time (0.33 while
            # filming). Two physics steps per frame and a real-time cap: learners see the
            # cell at the speed a real one would run.
            "physics_dt": 1.0 / 60.0, "rendering_dt": 1.0 / 30.0, "max_rtf": 1.0,
            # Matte: it still carries the physics and catches shadows, but the warehouse's
            # own floor is what the cameras see.
            "ground": {"kind": "plane", "size": 40.0, "color": [0.27, 0.27, 0.29],
                       "friction": 0.6, "matte": True},
            "render": {"viewport_grid": False},
            "lighting": {
                # A factory, brightly lit (Brandon, 2026-10-19): the dome at 3000 (was 520)
                # lights the hall the overview sees, and six high-bay discs 5 m up over the
                # cell (HIGH_BAYS) light the line. Measured on the dev cell, every learner
                # camera at the view it is used for: overview mean 33 -> 96, the tunnel cameras
                # 48-68 -> 125-134, realsense 74 -> 118, grasp-cam 65 -> 135, fisheye 159 ->
                # 162; p99 at most 235, nothing above 0.07 % clipped.
                "dome": {"intensity": 3000, "color": [0.92, 0.95, 1.0]},
                "sphere_intensity": 75000,
                # The inspection flashes, each riding the flange its camera rides.
                #
                # The interior shot came back BLACK, which is the correct image of an
                # unlit cavity: the fisheye sits 70 mm through a 160 x 216 mm aperture
                # with the dome outside it. The flag-hole shot has the same problem in a
                # milder form - an 8 mm hole in black paint, lit only by a room.
                #
                # Just behind each lens rather than on it, so the lamp is not in its own
                # frame, and `visibleInPrimaryRay` off so it never renders as a white ball
                # inside the shot it is lighting.
                "lamps": [
                    # (arm-a's flash is the fisheye tool's own ring light now, driven
                    # by the module from the asset's `flash:onIntensity`.)
                    {"name": "realsense-lamp", "kind": "sphere",
                     "parent_prim": FLANGE.format(prim="arm_b"),
                     # BACK on the tool axis, and much dimmer. The shell's paint is very
                     # nearly pure specular at this range: a raycast from the lens goes
                     # clean through the 8 mm hole and hits the far wall 439 mm away, so
                     # the station's aim is exact - and the wall still rendered black with
                     # the lamp 90 mm off-axis at 60000, while 12000 ON-axis blew it to
                     # white. Both readings say the same thing. The only light that comes
                     # back off a mirror is the light the mirror is pointing at, and for a
                     # camera that is the axis it looks down. So the lamp belongs on the
                     # axis, as a RING - wide enough not to burn a point reflection, and
                     # dim enough to land inside the sensor's range.
                     # Opposite side from the camera. With the lens off the tool axis the
                     # specular lobe no longer comes back down it, so the lamp is lighting
                     # the wall obliquely and the matte finish is what makes that work.
                     "local_position": [round((TOOL_LENS["realsense"] + 20.0) / 1000.0,
                                              4), 0.0, 0.09],
                     "intensity": 18000.0, "radius_m": 0.05,
                     "color": [1.0, 1.0, 1.0]},
                    {**FISHEYE_LAMP, "parent_prim": FLANGE.format(prim="arm_a")},
                    *HIGH_BAYS,
                ],
            },
            "props": props,
        },
    }]

    roles = {v: k for k, v in THREE.items()}
    homes = _homes()
    for name in sorted(cell.arm_bases_mm):
        base = cell.arm_bases_mm[name]
        prim = name.replace("-", "_")
        components.append({
            "name": name, "namespace": "rdk", "type": "arm",
            "model": "viam-education:isaac-sim:arm",
            "frame": {"parent": "world",
                      "translation": {"x": base[0], "y": base[1], "z": base[2]},
                      "orientation": {"type": "ov_degrees",
                                      "value": {"x": 0, "y": 0, "z": 1,
                                                "th": round(cell.arm_yaw_deg[name], 2)}}},
            "attributes": {
                "world": "sim-world", "asset": ARM_ASSET,
                "position": _m(base),
                "yaw_deg": round(cell.arm_yaw_deg[name], 2),
                "end_effector_prim": FLANGE.format(prim=prim),
                # Gains and limits carried over from the two-arm cell, where they were
                # measured rather than guessed: stiffness 200000 with damping 2000 is what
                # brought the standing error under load from 1.03 degrees to 0.10, and the
                # 1000 N.m ceiling is what the set-down needed when j1 pinned at 300.
                "drive_stiffness": 200000.0, "drive_damping": 2000.0,
                "drive_max_effort": 1000.0,
                "home_joints_deg": homes[name],
                # 40 deg/s was a SUCTION CUP's ceiling, measured on the two-arm cell
                # carrying a 2.35 kg mailbox on four cups: above it the part slid. Nothing
                # in this cell is held that way any more - the flag and the pin are held by
                # a fabricated joint between jaws, and the shell is held by its fixture -
                # so the limit that set 40 does not apply and the cycle was paying for it
                # at every one of eleven stations.
                "joint_speed_deg_s": 100.0, "joint_accel_deg_s2": 350.0,
                # Wider than the previous cell's +-180, because here the clamp was
                # causing the failures it was meant to prevent.
                #
                # The clamp lives in the PLANNER's kinematic model; the sim's joints carry
                # the asset's own wider limits, so a drive can be commanded past the clamp
                # and come to rest outside it. From there IK cannot even seed, and it does
                # not say so legibly - it reports "nlopt: INVALID_ARGS" ten times over.
                # Measured, arm-a came to rest at j3 207 and j5 -301 after one failed move.
                #
                # The taught poses need j1 to -241 and j3 to +166, so these cover the work
                # with margin while still being narrow enough to stop the planner reaching
                # a pose by the long way round, which is what the clamp is actually for.
                "joint_limit_deg": 200,
                "joint_limits_deg": {"shoulder_lift_joint": [-275, 95],
                                     "wrist_2_joint": [-275, 275],
                                     "wrist_3_joint": [-205, 205]},
                "_role": roles[name],
            },
        })

    # The DOOR arrives fitted and lying FLAT, open 90 degrees about its hinge, on the
    # pallet's door shelf. Its pose is the asset's published `flatPosition`/`flatOrientation`.
    door = part("Door")
    low, high = door.bounds_mm
    props.append({
        "name": "door", "type": "usd", "usd_path": door.usd_path, "prim": door.prim,
        "fixed": False, "position": _m(indexed_mm(cell, "door", start)),
        "orientation_wxyz": [float(v) for v in door.at("assembly:flatOrientation")],
        "scale": 1.0, "articulation_root": False, "body_prim": door.prim,
        "bounds_m": [_flat_door_bounds(door)],
    })
    # The FEEDERS. The pin stands head-up in a nest beside the belt and the flag rides its
    # own angled belt up to a stop: see fixture.FEEDERS. Both are fixed points, so neither
    # part rides the pallet any more.
    for bname, centre, dims, yaw in feeder_boxes():
        prop = {"name": bname, "type": "cube", "fixed": True, "size": 1.0,
                "scale": _m(dims), "position": _m(centre),
                "color": ([0.12, 0.12, 0.13] if bname in ("flag_feed", "flag_feed_b")
                          else [0.62, 0.64, 0.67])}
        if yaw:
            prop["yaw_deg"] = yaw
        if bname in ("flag_feed", "flag_feed_b"):
            prop["friction"] = 0.9      # the chute: static now, a nest at its end
        props.append(prop)
    # The pin MAGAZINE, and the queue in it: slot 0 is the live pin (below), slots 1..7
    # static pins - decorative, since pins in resting contact cannot be teleported.
    mag_lo = [v * 1000.0 for v in mag_attr("asset:envelopeMin")]
    mag_hi = [v * 1000.0 for v in mag_attr("asset:envelopeMax")]
    props.append({
        "name": "pin_magazine", "type": "usd", "usd_path": MAG_USD, "prim": "/PinMagazine",
        "fixed": True, "articulation_root": False, "scale": 1.0,
        "position": _m(_mag_origin_mm()), "yaw_deg": 0.0,
        "kinematic_parts": ["Lifter"],
        "bounds_m": [{"name": "magazine", "min": _m(mag_lo), "max": _m(mag_hi)}],
    })
    pin_part = part("Pin")
    pin_rot = [float(v) for v in mag_attr("magazine:pinOrientation")]
    for k, seat in enumerate(mag_slots_mm()[1:], start=1):
        props.append({
            "name": f"pin_q{k}", "type": "usd", "usd_path": pin_part.usd_path,
            "prim": pin_part.prim, "fixed": True, "position": _m(seat),
            "orientation_wxyz": pin_rot, "scale": 1.0, "articulation_root": False,
            "body_prim": pin_part.prim,
            "bounds_m": [dict(zip(("name", "min", "max"), (f"pin_q{k}", *map(_m, _turned_bounds(
                pin_part.bounds_mm[0], pin_part.bounds_mm[1], pin_rot)))))],
        })
    flag_part = part("Flag")
    flag_rot = [round(float(v), 6) for v in feed_orientation("Flag")]
    for k, at in enumerate(flag_queue_mm(cell), start=1):
        props.append({
            "name": f"flag_q{k}", "type": "usd", "usd_path": flag_part.usd_path,
            "prim": flag_part.prim, "fixed": True, "position": _m(at),
            "orientation_wxyz": flag_rot, "scale": 1.0, "articulation_root": False,
            "body_prim": flag_part.prim,
            "bounds_m": [dict(zip(("name", "min", "max"), (f"flag_q{k}", *map(_m, _turned_bounds(
                flag_part.bounds_mm[0], flag_part.bounds_mm[1], flag_rot)))))],
        })
    for name, part_name in (("flag", "Flag"), ("pin", "Pin")):
        pp = part(part_name)
        low, high = pp.bounds_mm
        where, orient = feed_spawn(cell, part_name)
        props.append({
            "name": name, "type": "usd", "usd_path": pp.usd_path, "prim": pp.prim,
            "fixed": False, "position": _m(where),
            "orientation_wxyz": [round(float(v), 6) for v in orient],
            "scale": 1.0, "articulation_root": False, "body_prim": pp.prim,
            "bounds_m": [dict(zip(("name", "min", "max"),
                                  (name, *map(_m, _turned_bounds(low, high, orient)))))],
            **({"never_sleep": True} if name == "flag" else {}),
        })

    # The tools. Only a suction gripper exists in this module, so the jaw grippers the
    # sequence wants are cups for now - which the flag's 60 mm plate, the door's flat
    # panel and the shell's 128 mm side wall all take, and which the previous cell proved
    # on exactly that side wall. The pin's 8 mm head is the one that will want real
    # fingers, and it is noted rather than pretended.
    for arm, tool in (("arm-b", "jaw"), ("arm-c", "big")):
        prim = arm.replace("-", "_")
        components.append({
            "name": f"grip-{arm[-1]}", "namespace": "rdk", "type": "gripper",
            "model": "viam-education:isaac-sim:gripper",
            "attributes": {
                "world": "sim-world", "parent_prim": FLANGE.format(prim=prim),
                "offset": [round(TOOL_STANDOFF[tool] / 1000.0, 4), 0.0, 0.0],
                # The jaw's attach ray now has ~21 mm to travel to the collar's top: the
                # pads centre on the grasp point (stations.JAW_PAD_MM), 28.5 mm past the
                # 134 mm attach point.
                # 30 mm: the attach point is 160 mm out (stations.JAW_ATTACH_MM), ~20 mm short
                # of the knob and collar tops.
                "max_grip_distance": 0.03 if arm == "arm-b" else 0.02,
                # The envelope the PLANNER is given, and it has to be the real tool's.
                #
                # The asset's own colliders are stripped when it is referenced - a
                # collider on a finger fights the fabricated joint that does the holding -
                # so this is the only thing standing between the planner and a tool it
                # cannot see. Left at the stand-in's 22 mm the planner routed a 75 mm
                # Hand-E straight through the parts tray: measured, the pin ended up
                # 145 mm off its slot and the flag on its side, both knocked off by a
                # tool nothing in the model knew the size of.
                #
                # Circumscribing radius, from the mirrored assets: Hand-E is 75 x 75 mm
                # in section over 151 mm, the UR suction unit 106 x 100 over 161.
                "geometry": {"radius_mm": EGP_ENVELOPE_MM[0] if tool == "jaw" else 53,
                             "length_mm": EGP_ENVELOPE_MM[1] if tool == "jaw" else 168.0},
                # THE REAL TOOL, referenced from the vendor's own asset.
                #
                # Two cuboids stood in for a gripper here, and they were honest about one
                # thing - that something holds the part - and wrong about everything else.
                # Worse, BOTH arms wore a suction cup with cuboid fingers drawn over it,
                # which is not a machine anyone could build: a cup cannot grip the end of
                # an 8 mm pin, and two fingers cannot span a 167 mm shell. One tool per
                # arm, each the one the job actually calls for, and no Y mount because no
                # arm needs two.
                #
                # Referenced VISUAL ONLY - the module strips the articulation, the rigid
                # bodies and the colliders - because both assets ship as working robots
                # and an articulation nested inside the arm's is not a thing PhysX allows.
                # The hold is still the surface gripper's fabricated joint.
                "jaw": (EGP if tool == "jaw" else SUCTION),
            },
        })

    # The wrist cameras, children of their arms' flanges so they travel with them.
    #
    # arm-b's camera sits BESIDE its tool, not behind it. Behind it, every frame it took was
    # a portrait of its own suction cup: the cup is 22 mm in radius and stands 135 mm in
    # front of the lens, which at a 69 degree field fills a quarter of the frame, dead
    # centre, exactly where the flag hole is. It cost a wrong conclusion before it was
    # found - the wall "rendering black on a white mailbox" was read as the paint being a
    # pure mirror, and the wall was never in the picture at all.
    for name, arm, tool, fov in (("fisheye-cam", "arm-a", "fisheye", 150.0),
                                 ("realsense-cam", "arm-b", "realsense", 69.0)):
        prim = arm.replace("-", "_")
        side = CAM_SIDE_MM.get(name, 0.0)
        # Offset to the side, then aimed back in at the work point, so the station's own
        # 320 mm standoff still puts the hole in the middle of the frame.
        # The leg forward is from the LENS to the work, not from the flange: the lens is
        # already TOOL_LENS down the tool axis, so the triangle is shorter than the
        # standoff by exactly that.
        pitch = 90.0 - math.degrees(math.atan2(
            abs(side), TOOL_STANDOFF[tool] - TOOL_LENS[tool]))
        components.append({
            "name": name, "namespace": "rdk", "type": "camera",
            "model": "viam-education:isaac-sim:camera",
            **({"frame": REALSENSE_FRAME} if name == "realsense-cam" else {}),
            "attributes": {
                "world": "sim-world", "parent_prim": FLANGE.format(prim=prim),
                # The LENS offset, not the standoff. For the fisheye they happen to be
                # the same 190 mm because the lens is at the stem's tip; for the RealSense
                # they are not, and using the standoff put the lens 320 mm down the tool
                # axis - which is exactly the flag hole, inside the shell's wall.
                "local_position": [round(TOOL_LENS[tool] / 1000.0, 4), 0.0,
                                   round(side / 1000.0, 4)],
                # -Z forward, and the two docstrings are describing different code
                # paths rather than contradicting each other. A camera given `target` is
                # posed with `camera_axes="world"` (+X forward); a camera given
                # `local_orientation_rpy_deg` is posed with `camera_axes="usd"`, where
                # forward is the prim's -Z - which the module's own comment says, and
                # which the overhead camera corroborates by reporting z_axis (0,0,1)
                # while looking straight down.
                #
                # With R = Rz*Ry*Rx (spatial.quat_from_euler_deg), [180,90,0] sends -Z to
                # local +X - the tool axis, which the GRIPPER docstring is the authority
                # on: "the flange prim's local +x pointing down, so the tool runs along
                # +x there". It also sends the image's up to local -Y, which at these
                # poses is world +Z, so frames come out the right way up.
                "local_orientation_rpy_deg": [180.0, round(pitch, 2), 0.0],
                "fov_deg": fov, "width": 1280, "height": 720,
                **({"fisheye": FISHEYE_LENS, "tool": FISHEYE_TOOL}
                   if name == "fisheye-cam" else {"tool": WRIST_D4XX}),
            },
        })

    # (No overhead camera: Brandon combined the infeed classification and the paint check
    # at the vision tunnel, 2026-10-13. `tunnel-top` does the classification.)
    # The tunnel's three cameras are the asset's own camera prims, so each sits and looks
    # exactly where the usd session framed it (the whole shell, door, pin and flag at
    # 1280x720). No `frame`: the paint check measures nothing in millimetres.
    if "tunnel" in cell.index_x_for:
        for cam_name, (rel, fov) in TUNNEL_CAMERAS.items():
            components.append({
                "name": cam_name, "namespace": "rdk", "type": "camera",
                "model": "viam-education:isaac-sim:camera",
                "attributes": {"world": "sim-world",
                               "prim_path": f"/World/vision_gantry/{rel}",
                               "fov_deg": fov, "width": 1280, "height": 720},
            })
    # DETAIL cameras, for the film's inset. From the overview a Ø3.4 mm pin and a 6 mm peg
    # are a pixel or two; "the parts go in" has to be SEEN, so each moment that matters has
    # a camera close enough to show it - the two feeders, the hinge and the flag hole.
    for cam_name, want, target, fov in detail_cameras(cell):
        body, at, aim = mounted_camera(cam_name, want, target)
        props.extend(body)
        components.append({
            "name": cam_name, "namespace": "rdk", "type": "camera",
            "model": "viam-education:isaac-sim:camera",
            "frame": {"parent": "world", "translation": {
                "x": round(at[0], 1), "y": round(at[1], 1), "z": round(at[2], 1)}},
            "attributes": {"world": "sim-world", "target": _m(aim),
                           "fov_deg": fov, "width": 640, "height": 360},
        })
    # The GRASP camera, for the course (page 8): straight DOWN on the handling stop, so it
    # sees where the reject stands and which way it is turned - the pose arm-c's grasp on
    # the -Y wall is worked out from. RGB only for now; depth and a point cloud come with R5.
    #
    # Top-down since 2026-10-13. Looking square at the -Y wall it needed a post between
    # arm-a and arm-c, and the planner found that post in both arms' paths (reach_check:
    # arm-a's inside and back shot, arm-c's grasp). It hangs from the same post-and-boom
    # fixture as overhead-cam, its post on the +Y side clear of arm-b, the boom turned to
    # reach over the belt.
    gx = cell.index_x("handle")
    _, gy, gz = cell.shell_origin_mm
    lens = (gx, cell.belt_y_mm, cell.belt_top_mm + 1150.0)
    # The fixture's boom runs along its own -Y: Rz(yaw)(0, -1) = (sin yaw, -cos yaw).
    yaw = math.degrees(math.atan2(GRASP_BOOM_DIR[0], -GRASP_BOOM_DIR[1]))
    props.append({
        "name": "grasp_cam_fixture", "type": "usd", "usd_path": OVERHEAD_FIXTURE_USD,
        "prim": "/OverheadCameraFixture", "fixed": True, "articulation_root": False,
        "position": _m((lens[0] - 700.0 * GRASP_BOOM_DIR[0], lens[1] - 700.0 * GRASP_BOOM_DIR[1],
                        lens[2] - 1800.0)),
        "yaw_deg": round(yaw, 2),
        "bounds_m": OVERHEAD_FIXTURE_BOUNDS_M,
    })
    components.append({
        "name": "grasp-cam", "namespace": "rdk", "type": "camera",
        "model": "viam-education:isaac-sim:camera",
        "frame": {"parent": "world", "translation": {
            "x": round(lens[0], 1), "y": round(lens[1], 1), "z": round(lens[2], 1)},
            "orientation": DOWN_CAMERA_OV},
        "attributes": {"world": "sim-world",
                       "target": _m((lens[0], lens[1], gz)),
                       "fov_deg": 50.0, "width": 1280, "height": 720,
                       # A depth camera: arm-c finds the reject from its point cloud
                       # (probes/assembly/shell_locate.py; Brandon, 2026-10-14).
                       "depth": True},
    })
    # And one standing back, for video.
    components.append({
        "name": "overview-cam", "namespace": "rdk", "type": "camera",
        "model": "viam-education:isaac-sim:camera",
        # ON THE SIDE THE WORK HAPPENS, and aimed at the middle of it.
        #
        # Both faults were found by watching the film back rather than by reading the
        # config, which is the whole argument for `probes/assembly/review_film.py`.
        #
        # The camera stood at y -2400. arm-b is at y +800 and the carrier's two slots are
        # at y +220 and +290 - so the assemble arm and both parts it fits were BEHIND the
        # mailbox in every frame. arm-b does five of the twelve stations, including both
        # picks and both inserts, and not one of them was visible. A film of an assembly
        # cell that does not show the assembly is not a film of the cell.
        #
        # And it was aimed at x -55, the FIRST index, while the stations run to x 901 - so
        # the cycle happened in the right-hand two thirds of the frame with the left third
        # on empty floor, at a 52 degree field 3.5 m out: a 3.4 m wide shot of a 1 m
        # process.
        #
        # Moved to the assemble side and lifted, aimed at the mid-index and narrowed to
        # 38 degrees, the frame is about 2.6 m across - still the whole 956 mm of belt
        # travel and all three arms, with the work roughly twice the size and on the near
        # side of it. arm-c's handling is then the far side, which is the right trade:
        # it is four stations of carrying, against five of the assembly the course is about.
        # Lower (2.1 m, not 2.6) since the warehouse: from higher up the background was
        # all floor; from here the racks stand behind the cell.
        "frame": {"parent": "world", "translation": {"x": 2600, "y": 2600, "z": 2100}},
        "attributes": {"world": "sim-world",
                       # The WHOLE line: from the tunnel to the reject spur, so the
                       # paint check and arm-c's set-down are in shot too.
                       "target": _m((((cell.index_x("tunnel") if "tunnel" in cell.index_x_for
                                       else cell.index_x("inside"))
                                      + cell.repaint_x_mm) / 2.0, -500.0,
                                     cell.belt_top_mm + 150.0)),
                       # Rendered only while read, like every camera: always-on it cost
                       # 9 ms of every frame whether or not anything was filming.
                       "fov_deg": 50.0, "width": 1280, "height": 720},
    })

    return {
        "_comment": [
            "GENERATED by probes/assembly/build_fragment.py from the cell description.",
            "Do not hand-edit: tests/test_assembly_fragment.py regenerates this and fails",
            "on any difference, because a hand edit is how one source becomes two.",
        ],
        "modules": [{"type": "local", "name": "isaac-sim",
                     "executable_path": "/home/shrews/viam/viam-isaac-sim/run.sh"}]
                   + _vision()["modules"],
        # The learner's surface (viam-103 R1): the cell's verbs and the batch's score.
        "components": components + [
            {"name": "scoreboard", "type": "sensor", "namespace": "rdk",
             "model": "viam-education:isaac-sim:scoreboard", "attributes": {"cell": "cell"}}],
        "services": _vision()["services"] + [
            {"name": "cell", "type": "generic", "namespace": "rdk", "model": "viam-education:isaac-sim:cell",
             "attributes": {"motion": True}}]
        # Page 10's `--vision oracle`: what a perfect model answers, from the cell's truth.
        + [{"name": f"{kind}-oracle", "type": "vision", "namespace": "rdk",
            "model": "viam-education:isaac-sim:oracle", "attributes": {"kind": kind, "cell": "cell"}}
           for kind in ("infeed", "inside", "features", "shell", "paint")],
        "packages": _vision()["packages"],
    }


# A park configuration per arm, and why the fragment cannot do without one.
#
# The first generated fragment had none, and the consequence was not a missing convenience:
# `reset` had nowhere to send the arms, so a run that jammed an arm left it jammed. arm-a
# ended at j3 207 degrees and j5 -301, both OUTSIDE its own configured limits, and from
# there the IK solver does not refuse legibly - it reports "nlopt: INVALID_ARGS", because
# the seed it was handed is out of bounds. An arm that cannot be parked cannot be recovered.
#
# Park is the shape the arm WORKS in with its shoulder lifted, which is the previous cell's
# hard-won rule: a park solved for as a cartesian goal comes out a different shape on
# different days, and the move away from it then unwinds hundreds of degrees. So it is
# derived from a pose the arm is known to reach, and the only change is the shoulder.
SAFE_HOME = [0.0, -140.0, 60.0, -100.0, -90.0, 0.0]
SHOULDER_LIFT_DEG = 30.0


def _unwound(joints: List[float]) -> List[float]:
    """The same arm pose with its wrists inside +-180.

    A taught pose can carry a wrist wound past half a turn - the planner is free to return
    one, and the park it returned for arm-a sat at wrist_2 -270 against a -275 clamp. A
    revolute joint turned by a whole revolution is the SAME flange pose, so taking the
    equivalent angle costs nothing and parks the arm with room on both sides. A park with
    5 degrees of margin is a park that a drive overshooting by 6 turns into an
    "nlopt: INVALID_ARGS" with nothing in the message to say why.
    """
    out = list(joints)
    for j in (4, 5):
        while out[j] > 180.0:
            out[j] -= 360.0
        while out[j] < -180.0:
            out[j] += 360.0
    return out


def _homes() -> Dict[str, List[float]]:
    """Each arm's park configuration.

    A TAUGHT park if there is one - `park.json`, which the teaching pass asks the planner
    for directly: up and back over the arm's own base, tool down, clear of the belt and of
    the other arms, and verified against a cleared cell.

    The fallback below is what this used to do for everything, and it is kept only as a
    fallback: deriving the park from whichever working pose sorted first alphabetically
    parked arm-a with its tool hanging over the belt at the back-shot station, and adding
    the stand-off poses silently changed which file that was.
    """
    from .reach_check import POSES
    out: Dict[str, List[float]] = {}
    for arm in ("arm-a", "arm-b", "arm-c"):
        folder = POSES / arm
        park = folder / "park.json"
        if park.exists():
            out[arm] = _unwound(list(json.loads(park.read_text())["joints_deg"]))
            continue
        picked = sorted(p for p in folder.glob("*.json")
                        if ".clear." not in p.name) if folder.exists() else []
        if not picked:
            out[arm] = list(SAFE_HOME)
            continue
        joints = _unwound(list(json.loads(picked[0].read_text())["joints_deg"]))
        # (the wrists are already unwound by `_unwound`)
        # Unwind the wrists before lifting the shoulder.
        #
        # A taught pose can carry a wrist wound past half a turn - arm-a's back shot sits
        # at wrist_2 220.6 - and a park that inherits the winding inherits its problems:
        # the arm came to rest 55 degrees short of that park with 0.7 N.m on the joint,
        # which is a joint that is not being driven rather than one being held back. A
        # revolute joint turned by a whole revolution is the SAME flange pose, so taking
        # the equivalent angle inside +-180 costs nothing and parks the arm unwound.
        joints[1] = max(-265.0, min(85.0, joints[1] + SHOULDER_LIFT_DEG))
        out[arm] = [round(v, 3) for v in joints]
    return out


# Where the fragment's files are (R2). `local` is this workstation and the VM, which has the
# same trees at the same paths. `package` is a learner's machine: everything under the
# cell's archive package (scripts/build_cell_assets.py builds it, same layout), the
# warehouse streamed from Isaac 5.0's own asset server, and the module from the registry.
# `dir:PATH` is `package` with a literal directory in place of the package - the archive
# unpacked somewhere, to test a packaged fragment without uploading anything.
ASSET_TREES = {
    "/home/shrews/viam/usd/": "usd/",
    str(Path(__file__).resolve().parents[2] / "assets") + "/": "sim/",
    "/home/shrews/viam/isaac-package-shelf/assets/mirror/": "isaac/",
}
ISAAC_ASSET_ROOT = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
                    "Assets/Isaac/5.0/Isaac/")
STREAMED = ("Environments/Simple_Warehouse/",)


def canonical_path(path: str) -> str:
    """A relocated asset path back as this workstation names it, so where the files sit
    is not geometry: the layout hash reads usd paths, and moving a file moves nothing."""
    if not isinstance(path, str):
        return path
    mirror = "/home/shrews/viam/isaac-package-shelf/assets/mirror/"
    if path.startswith(ISAAC_ASSET_ROOT):
        return mirror + path[len(ISAAC_ASSET_ROOT):]
    for here, there in ASSET_TREES.items():
        marker = "/" + there                      # "/usd/", "/sim/", "/isaac/"
        if not path.startswith(here) and marker in path:
            return here + path.split(marker, 1)[1]
    return path


def relocate(fragment: dict, root: str, package: dict = None, module: dict = None) -> dict:
    """The fragment with every local path under `root` (or streamed), the cell told where
    the usd tree is, and - for a real package - the package and registry module entries."""
    def move(v):
        if isinstance(v, dict):
            return {k: move(x) for k, x in v.items()}
        if isinstance(v, list):
            return [move(x) for x in v]
        if isinstance(v, str):
            mirror = "/home/shrews/viam/isaac-package-shelf/assets/mirror/"
            for s_ in STREAMED:
                if v.startswith(mirror + s_):
                    return ISAAC_ASSET_ROOT + v[len(mirror):]
            for here, there in ASSET_TREES.items():
                if v.startswith(here):
                    return f"{root}/{there}{v[len(here):]}"
        return v
    out = move(fragment)
    for svc in out["services"]:
        if svc["name"] == "cell":
            svc["attributes"]["assets_dir"] = f"{root}/usd"
    if package:
        # A learner's machine runs the course's vision services, which course-tools adds per
        # page; the cell's own infeed classifier (a model package in another org, and the
        # tflite module that runs it) is the film's, not the learner's.
        own = {s_["name"] for s_ in _vision()["services"]}
        out["services"] = [s_ for s_ in out["services"] if s_["name"] not in own]
        out["modules"] = [m for m in out["modules"] if m["name"] == "isaac-sim"]
        out["packages"] = [package]
    if module:
        out["modules"] = [module if m["name"] == "isaac-sim" else m for m in out["modules"]]
    return out


def main() -> int:
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--assets", default="local",
                    help="local | package | dir:PATH (see ASSET_TREES above)")
    ap.add_argument("--package", default="cell-assets", help="the archive's name in the config")
    ap.add_argument("--package-id", default="ORG_ID/viam-103-cell-assets",
                    help="ORG/NAME of the uploaded archive")
    ap.add_argument("--package-version", default="latest")
    ap.add_argument("--module-id", default="ORG_NAMESPACE:isaac-sim")
    ap.add_argument("--module-version", default="latest")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    if os.environ.get("CELL_USD_DIR"):
        # The asset readers follow it (asset.FRAMES, fixture.MAG_USD), so a fragment built
        # with it set names that tree as if it were the workstation's - which is how a
        # verification run's test directory leaked into the VM's live config (2026-10-18).
        raise SystemExit("unset CELL_USD_DIR: the fragment is built from the workstation's "
                         "tree; use --assets dir:PATH to point it somewhere else")
    cell = chosen()
    frag = build(cell)
    out = FRAGMENT
    if args.assets == "package":
        frag = relocate(frag, "${packages.archive." + args.package + "}",
                        package={"name": args.package, "package": args.package_id,
                                 "version": args.package_version, "type": "archive"},
                        module={"type": "registry", "name": "isaac-sim",
                                "module_id": args.module_id, "version": args.module_version,
                                # first_run.sh installs Isaac Sim (10 GB+) on first start
                                "first_run_timeout": "2h0m0s"})
        out = FRAGMENT.with_name("assembly-cell-package.json")
    elif args.assets.startswith("dir:"):
        frag = relocate(frag, args.assets[len("dir:"):].rstrip("/"))
        out = FRAGMENT.with_name("assembly-cell-dir.json")
    if args.out:
        out = Path(args.out)
    out.write_text(json.dumps(frag, indent=2) + "\n")
    print(f"wrote {out.name}: {len(frag['components'])} components, "
          f"{len(frag['components'][0]['attributes']['props'])} props")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
