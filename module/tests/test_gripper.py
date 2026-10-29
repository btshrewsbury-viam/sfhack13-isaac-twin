"""The suction gripper, exercised on the mock backend.

What is worth pinning here is the behaviour that differs from a kinematic attach, because
that difference is the whole reason to use isaac's surface gripper: a grab reports whether
it actually caught something, and it can say no.
"""

import asyncio

import pytest
from viam.proto.app.robot import ComponentConfig
from viam.utils import dict_to_struct

from isaac_module.models.gripper import IsaacGripper
from isaac_module.sim_manager import MockGripperHandle, SimConfig, SimManager


def config(**attrs):
    base = {"world": "sim-world", "parent_prim": "/World/arm_a/wrist_3_link/flange"}
    base.update(attrs)
    return ComponentConfig(name="suction", attributes=dict_to_struct(base))


@pytest.fixture
def gripper():
    manager = SimManager.get()
    previous_cfg, previous_mock = manager.cfg, manager.mock
    manager.cfg = SimConfig(mock=True)
    manager.mock = True
    manager._booted.set()
    manager._handles.pop("suction", None)
    try:
        yield IsaacGripper.new(config(grab_settle_sec=0.05), {})
    finally:
        manager._handles.pop("suction", None)
        manager.cfg, manager.mock = previous_cfg, previous_mock
        manager._booted.clear()


def test_parent_prim_is_required():
    """A cup has to hang off something; without it the joint has no body to anchor to."""
    bad = ComponentConfig(name="suction", attributes=dict_to_struct({"world": "w"}))
    with pytest.raises(ValueError, match="parent_prim"):
        IsaacGripper.validate_config(bad)


def test_starts_open(gripper):
    assert asyncio.run(gripper.is_moving()) is False
    assert asyncio.run(gripper.is_holding_something()).is_holding_something is False


def test_grab_reports_failure_when_nothing_is_there(gripper):
    """The mock grips nothing, on purpose.

    A mock that always reported success would let a whole round pass without a
    simulator and fail the first time it met one.
    """
    assert asyncio.run(gripper.grab()) is False


def test_open_after_close_returns_to_open(gripper):
    asyncio.run(gripper.grab())
    asyncio.run(gripper.open())
    assert asyncio.run(gripper.is_moving()) is False


def test_stop_does_not_drop_what_is_held():
    """stop() means "stop trying", not "let go" - a held part must stay held."""
    handle = MockGripperHandle("suction", {})
    handle.close()
    assert handle.status() == "Closed"


def test_geometry_is_reported_for_the_planner(gripper):
    """The cup is in neither the kinematics file nor the articulation.

    Unreported, a path clears the flange by a millimetre and puts the cup through the
    bench, with nothing to collide against.
    """
    geometries = asyncio.run(gripper.get_geometries())
    assert len(geometries) == 1
    assert geometries[0].capsule.length_mm > 0
    assert geometries[0].label.endswith("-cup")


def test_geometry_follows_the_configured_offset():
    manager = SimManager.get()
    previous_cfg, previous_mock = manager.cfg, manager.mock
    manager.cfg, manager.mock = SimConfig(mock=True), True
    manager._booted.set()
    manager._handles.pop("suction", None)
    try:
        # The offset is in the PARENT prim's frame, where the tool is +x - which is how
        # the cell configures it. The geometry is reported in the viam frame, where the
        # tool is +z. Copying components across put the cup 120 mm sideways; only the
        # magnitude carries between the two frames.
        g = IsaacGripper.new(
            config(offset=[0.12, 0.0, 0.0], geometry={"radius_mm": 10}), {}
        )
        geom = asyncio.run(g.get_geometries())[0]
        assert geom.capsule.radius_mm == 10
        assert geom.capsule.length_mm == pytest.approx(120.0)
        assert geom.center.x == 0.0 and geom.center.y == 0.0, \
            "the cup must lie on the viam tool axis, not on the parent frame's +x"
        assert geom.center.z == pytest.approx(60.0)
    finally:
        manager._handles.pop("suction", None)
        manager.cfg, manager.mock = previous_cfg, previous_mock
        manager._booted.clear()


def test_joints_are_refused():
    """A suction cup has no controllable joints; pretending otherwise invents state."""
    g = IsaacGripper("suction")
    with pytest.raises(NotImplementedError):
        asyncio.run(g.get_current_inputs())


# ---- the limits the prim is authored with ------------------------------------


def test_absent_shear_attribute_takes_the_derived_array_capacity():
    """An omitted limit must fall back to what this module derives, not to a literal.

    This is the regression that cost a carry. `sim_manager` restated the fallbacks as
    its own literals - 50 N of shear - which shadowed the 306 N derived here from the
    cup diameter and the vacuum an EPick pulls. Nothing failed loudly: the cell simply
    ran on a sixth of the tool it claimed to model, and the first long carry threw the
    part across the cell. So what is pinned is not the number but the SOURCE of it.
    """
    from isaac_module.surface_gripper import (
        DEFAULT_SHEAR_FORCE_LIMIT_N, EPICK, cup_vacuum_force_n, gripper_limits)

    limits = gripper_limits({})
    assert limits.shear_force_limit_n == pytest.approx(DEFAULT_SHEAR_FORCE_LIMIT_N)
    # and that derived value is the array's, not one cup's
    assert limits.shear_force_limit_n == pytest.approx(
        cup_vacuum_force_n() * 0.5 * len(EPICK["cups"]["names"]))
    assert limits.shear_force_limit_n > 300.0


def test_a_stated_shear_limit_still_wins():
    from isaac_module.surface_gripper import gripper_limits

    assert gripper_limits({"shear_force_limit": 42}).shear_force_limit_n == 42.0


def test_the_coaxial_check_stays_off_by_default():
    """Deliberate, and documented in `gripper_limits`: the plugin's coaxial reading is a
    single physics step with no averaging, so a real ceiling fires on the spikes a
    waypointed linear move puts into stiff drives."""
    from isaac_module.surface_gripper import gripper_limits

    assert gripper_limits({}).coaxial_force_limit_n == 0.0
