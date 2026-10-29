"""Fixtures for the tests that need the running cell.

One round per fixture, not one per claim. A test file with one test per claim over a
sequential round would run the round five times and fail all five identically on a bad
pick; running it once and writing the assertions over the trace names the failure.
"""

import asyncio
import os
import sys
from pathlib import Path

import pytest

# The module under test and the round driver both live outside tests/.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "probes"))

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")


def machine_available() -> bool:
    return bool(os.environ.get("VIAM_API_KEY") and os.environ.get("VIAM_API_KEY_ID"))


def pytest_configure(config):
    config.addinivalue_line(
        "markers", "machine: needs the running cell (skipped without VIAM_API_KEY)")


def pytest_collection_modifyitems(config, items):
    if machine_available():
        return
    skip = pytest.mark.skip(reason="no VIAM_API_KEY; the cell is not reachable")
    for item in items:
        if "machine" in item.keywords:
            item.add_marker(skip)


async def _connect():
    from qc.round import Clients
    from viam.components.arm import Arm
    from viam.components.camera import Camera
    from viam.components.generic import Generic
    from viam.components.gripper import Gripper
    from viam.robot.client import RobotClient
    from viam.services.motion import MotionClient

    robot = await RobotClient.at_address(
        FQDN, RobotClient.Options.with_api_key(
            api_key=os.environ["VIAM_API_KEY"],
            api_key_id=os.environ["VIAM_API_KEY_ID"]))
    return robot, Clients(
        motion=MotionClient.from_robot(robot, "builtin"),
        world=Generic.from_robot(robot, "sim-world"),
        arms={n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b")},
        grippers={"arm-a": Gripper.from_robot(robot, "suction-a"),
                  "arm-b": Gripper.from_robot(robot, "suction-b")},
        camera=Camera.from_robot(robot, "inspect-cam"),
        frames_base=Path(__file__).resolve().parents[1] / ".scratch" / "frames")


PROGRAM = Path(__file__).resolve().parents[1] / "fragments" / "qc-cell-program.json"


def _round(defects):
    """Run one round with a stated set of defects and return (trace, failure).

    Which tray the part ends in is the round's own answer. A fixture that named the
    tray would prove the arm can reach it and nothing about the check.

    The taught program is replayed when the cell has one, which is what the cell does
    in service. Testing the planned path instead would be testing something the cell
    only does when it is being taught.
    """
    import json

    from qc.layout import Cell
    from qc.round import Program, Trace, run_round

    async def go():
        robot, clients = await _connect()
        if PROGRAM.exists():
            clients.program = Program.from_json(json.loads(PROGRAM.read_text()))
        trace = Trace()
        failure = None
        try:
            await run_round(clients, Cell.from_fragment(), defects=defects,
                            trace=trace)
        except Exception as exc:  # noqa: BLE001
            # Keep the trace. A round that dies mid-way is exactly when the record of
            # what it managed is worth most, and every assertion below can then say
            # which step it got to rather than all failing on a connection error.
            failure = exc
        finally:
            await robot.close()
        return trace, failure

    return asyncio.run(go())


@pytest.fixture(scope="session")
def good_round():
    """A part with no marks on it. It should end in the good tray."""
    return _round({})


@pytest.fixture(scope="session")
def bad_round():
    """A part whose ONLY mark is on the face arm-a's cup covers.

    This is the round with teeth. The first check physically cannot see that face, so
    a cell that reached its verdict from arm-a alone sends this part to the good tray,
    and every other test in the suite still passes.
    """
    return _round({"defect_smudge": True})


@pytest.fixture(scope="session")
def cell():
    from qc.layout import Cell
    return Cell.from_fragment()
