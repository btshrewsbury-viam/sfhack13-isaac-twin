"""Ask the deployed classifier what it sees, with the cell staged to each class in turn.

The dataset says the model can separate these three classes on images; this says whether
the CELL can, through the vision service, on frames it has never seen. They are different
questions and only the second one matters: a model is deployed when a station can ask it a
question and act on the answer, not when a training job reports a number.

Staged by the same levers the capture used - the paint defect for good against damaged, the
shell's absence for the empty fixture - but at fixture positions the capture did not use, so
a model that memorised the training positions is caught here rather than in the cell.
"""
from __future__ import annotations

import asyncio
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from assembly.arms import clear_and_park                          # noqa: E402
from assembly.build_fragment import chosen                        # noqa: E402
from assembly.fixture import index_to, indexed_mm                 # noqa: E402
from viam.components.arm import Arm                               # noqa: E402
from viam.components.generic import Generic                       # noqa: E402
from viam.robot.client import RobotClient                         # noqa: E402
from viam.services.vision import VisionClient                     # noqa: E402

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
AWAY_MM = 2600.0
# Positions the capture never used, so memorising them is not a way to pass.
HELD_OUT_DX = (-12.0, 0.0, 12.0)
CASES = [
    ("good", "classic_black", "gloss", "none"),
    ("good", "poppy_garden", "matte", "none"),
    ("damaged", "federal_blue", "satin", "overspray"),
    ("damaged", "hot_rod_flames", "gloss", "overspray"),
    ("damaged", "classic_black", "satin", "overspray"),
    ("empty", "federal_blue", "gloss", "none"),
]


async def main() -> int:
    robot = await RobotClient.at_address(
        FQDN, RobotClient.Options.with_api_key(
            api_key=os.environ["VIAM_API_KEY"],
            api_key_id=os.environ["VIAM_API_KEY_ID"]))
    world = Generic.from_robot(robot, "sim-world")
    arms = {n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b", "arm-c")}
    vision = VisionClient.from_robot(robot, "infeed-vision")
    cell = chosen()
    base = cell.index_x("inside")

    right, total, rows = 0, 0, []
    at = None
    for want, scheme, finish, defect in CASES:
        await world.do_command({"command": "set_variants", "prop": "shell",
                                "variants": {"scheme": scheme, "finish": finish,
                                             "paint_defect": defect}})
        await world.do_command({"command": "reset"})
        await asyncio.sleep(1.8)
        await clear_and_park(world, arms, cell)
        at = None
        for dx in HELD_OUT_DX:
            at = await index_to(world, cell, base + dx, at)
            if want == "empty":
                await world.do_command({
                    "command": "place_props",
                    "props": [{"name": "shell",
                               "position_mm": indexed_mm(cell, "shell", AWAY_MM),
                               "yaw_deg": 0.0}]})
                await asyncio.sleep(0.5)
            got = await vision.get_classifications_from_camera("overhead-cam", 3)
            top = max(got, key=lambda c: c.confidence) if got else None
            ok = top is not None and top.class_name == want
            right += ok
            total += 1
            rows.append({"want": want, "scheme": scheme, "finish": finish,
                         "defect": defect, "dx_mm": dx,
                         "got": top.class_name if top else None,
                         "confidence": round(top.confidence, 3) if top else None,
                         "correct": bool(ok)})
            print(f"  {'OK ' if ok else 'BAD'} want {want:<8} got "
                  f"{(top.class_name if top else 'nothing'):<8} "
                  f"{(f'{top.confidence:.2f}' if top else '    ')}   "
                  f"{scheme} {finish} {defect} dx{dx:+.0f}", flush=True)

    out = Path(".scratch/runs/infeed-classify.json").resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({"right": right, "total": total, "rows": rows},
                              indent=1) + "\n")
    print(f"\n  {right}/{total} correct; wrote {out}")
    await robot.close()
    return 0 if right == total else 1


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
