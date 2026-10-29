"""Capture a labelled dataset for the overhead classifier.

Three classes, which is what the cell actually has to tell apart at the infeed: a good
mailbox, a damaged one, and an empty fixture. The asset gives the variation for free - four
paint schemes, four paint defects, three finishes - so the set spans what the classifier
will really see rather than one mailbox photographed repeatedly.

The empty class is not an afterthought. A fixture arriving with nothing in it is the most
common thing a real infeed sees after a jam, and a classifier trained only on mailboxes
answers "mailbox" to an empty saddle with complete confidence.

Variant switching recomposes the prim and invalidates every physics view in the scene,
including the arms', so it happens before `reset` and before anything moves - the order that
produced "random" degree-of-freedom failures in the previous build when it was wrong.
"""
from __future__ import annotations

import asyncio
import io
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from assembly.arms import clear_and_park                          # noqa: E402
from assembly.asset import part                                   # noqa: E402
from assembly.build_fragment import chosen                        # noqa: E402
from assembly.fixture import CARRIED, index_to, indexed_mm        # noqa: E402
from PIL import Image                                             # noqa: E402
from viam.components.arm import Arm                               # noqa: E402
from viam.components.camera import Camera                         # noqa: E402
from viam.components.generic import Generic                       # noqa: E402
from viam.robot.client import RobotClient                         # noqa: E402

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
OUT = Path(".scratch/datasets/infeed").resolve()
# Where the SHELL goes when the fixture is to arrive empty - the shell alone, not the
# carried set.
#
# The first version indexed everything away and photographed bare conveyor, which is a
# picture of an empty BELT and labels it "empty fixture". The class the cell actually has to
# recognise is a saddle that has arrived with nothing in it, which is what a jam upstream
# looks like, and a classifier trained on bare belt will not recognise it.
AWAY_MM = 2600.0
# How much the fixture's arrival varies, shot to shot. A conveyor stops where it stops: a
# set captured at one exact position teaches a classifier to expect one exact position, and
# the first thing that moves 10 mm reads as a new class.
JITTER_X_MM = (-18.0, -6.0, 6.0, 18.0)
DAMAGED_DEFECTS = tuple(
    os.environ.get("DAMAGED_DEFECTS", "overspray").split(","))


async def shot(camera, world, path: Path, label: str, meta: dict) -> None:
    images, _ = await camera.get_images()
    frame = Image.open(io.BytesIO(images[0].data)).convert("RGB")
    path.parent.mkdir(parents=True, exist_ok=True)
    frame.save(path, quality=92)
    path.with_suffix(".json").write_text(json.dumps(
        {"label": label, **meta}, indent=1, sort_keys=True) + "\n")


async def main() -> int:
    robot = await RobotClient.at_address(
        FQDN, RobotClient.Options.with_api_key(
            api_key=os.environ["VIAM_API_KEY"],
            api_key_id=os.environ["VIAM_API_KEY_ID"]))
    world = Generic.from_robot(robot, "sim-world")
    arms = {n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b", "arm-c")}
    over = Camera.from_robot(robot, "overhead-cam")
    cell = chosen()
    shell = part("Shell")
    schemes = shell.variant_sets["scheme"]
    defects = shell.variant_sets["paint_defect"]
    finishes = shell.variant_sets["finish"]
    at_index = None
    n = 0

    base = cell.index_x("inside")
    # Balanced by VARIETY, not just by frame count.
    #
    # The first balanced capture was 36 frames per class and still produced a classifier
    # that called every good mailbox damaged, at 0.54 to 0.75 confidence, with the empty
    # class perfect at 0.98. The counts were equal and the VARIETY was not: `good` is one
    # defect value, so it had 12 distinct configurations shot three times each, while
    # `damaged` had 36 distinct ones shot once. Three times the variety in one class is
    # three times the evidence, and the model learned "damaged" as the general case and
    # "good" as nothing in particular.
    #
    # So each class gets the same 12 configurations and the same 3 fixture positions. For
    # `damaged` that means choosing ONE defect per scheme-and-finish rather than all three,
    # rotated so all three defects are still represented across the set.
    # Which defects count as `damaged`, measured rather than assumed.
    #
    # At this camera's distance the three published defects are not equally visible, and the
    # difference is not marginal. Comparing each damaged frame with its good twin, the
    # number of pixels differing by more than 20 grey levels is:
    #     overspray   3200 to 4500
    #     drip        0 to 500, usually single digits
    #     smudge      0 to 420, usually single digits
    # Over the whole 1280x720 frame the average difference is 0.51 grey levels. Two thirds
    # of the damaged class was therefore invisible, which is why a classifier trained on it
    # called every GOOD mailbox damaged at 0.44 to 0.75 confidence while getting `empty`
    # right at 0.98 - it had no signal to learn, so it learned the prior.
    #
    # DAMAGED_DEFECTS is what this camera can actually see. Drip and smudge belong to a
    # closer look: this cell already takes one at the back cap from 90 mm and another with
    # the part lifted and turned under the paint camera.
    defect_cycle = [d for d in defects if d in DAMAGED_DEFECTS]
    plan = []
    for i, scheme in enumerate(schemes):
        for j, finish in enumerate(finishes):
            plan.append(("good", scheme, finish, "none"))
            plan.append(("damaged", scheme, finish,
                         defect_cycle[(i * len(finishes) + j) % len(defect_cycle)]))

    for label, scheme, finish, defect in plan:
        await world.do_command({
            "command": "set_variants", "prop": "shell",
            "variants": {"scheme": scheme, "finish": finish, "paint_defect": defect}})
        await world.do_command({"command": "reset"})
        await asyncio.sleep(1.8)
        await clear_and_park(world, arms, cell)
        at_index = None
        for dx in JITTER_X_MM[:3]:
            at_index = await index_to(world, cell, base + dx, at_index)
            meta = {"scheme": scheme, "finish": finish, "paint_defect": defect,
                    "camera": "overhead-cam", "cell": "assembly", "index_offset_mm": dx}
            await shot(over, world,
                       OUT / label / f"{scheme}_{finish}_{defect}_{dx:+.0f}.jpg",
                       label, meta)
            n += 1
        print(f"  {label:<8} {scheme:<15} {finish:<6} {defect:<10} x3", flush=True)

    # The empty FIXTURE: the saddle arrives, the shell does not. Only the shell is moved
    # away, so what the camera sees is a fixture with nothing in it. Twelve "configurations"
    # of an empty saddle is the same saddle, so its variety is the fixture's own position -
    # which is the only thing that actually varies for this class in the cell either.
    for i in range(12):
        for dx in JITTER_X_MM[:3]:
            at_index = await index_to(world, cell, base + dx + (i - 6) * 1.5, at_index)
            await world.do_command({
                "command": "place_props",
                "props": [{"name": "shell",
                           "position_mm": indexed_mm(cell, "shell", AWAY_MM),
                           "yaw_deg": 0.0}]})
            await asyncio.sleep(0.4)
            await shot(over, world, OUT / "empty" / f"fixture_{i:02d}{dx:+.0f}.jpg",
                       "empty",
                       {"camera": "overhead-cam", "cell": "assembly",
                        "index_offset_mm": dx + (i - 6) * 1.5,
                        "note": "saddle present, no shell"})
            n += 1
    print(f"  empty    fixture x36", flush=True)

    print(f"\n  {n} frames under {OUT}")
    await robot.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
