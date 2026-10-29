"""viam-education:isaac-sim:arm - a simulated arm.

Attributes:
  world (string, required)   - name of the viam-education:isaac-sim:world component
  asset (string)             - known robot, e.g. "ur20", "ur10", "franka"
  usd_path (string)          - explicit USD to spawn instead of a known asset
  prim_path (string)         - where to place it (default /World/<name>), or
                               an existing articulation in the stage
  position ([x,y,z] meters)  - spawn position
  end_effector_prim (string) - prim path whose world pose is reported by
                               GetEndPosition
  ignore_base_rotation (bool)- skip the known asset's fixed base rotation. UR assets
                               in isaac are URDF imports rooted at ROS base_link while
                               viam's kinematics use the controller's base frame, a half
                               turn apart, so the module composes that offset into the
                               spawn. Set this only for a USD you have already authored
                               in the kinematics frame.
  move_timeout_sec (float)   - max time to wait for a move (default 30)
  joint_limit_deg (float)    - NOTE: if an arm is already outside the narrowed range
                               when this is turned on, every motion.Move fails its start
                               check until the arm is driven back in bounds with
                               move_to_joint_positions, which is allowed to head inwards.
                               narrow every revolute joint's range in the kinematics
                               served to the motion service to +-this many degrees.
                               UR arms allow +-360, and the planner will happily pick
                               multi-turn solutions that accumulate until the arm jams;
                               180 covers every orientation and prevents that. SVA only
                               (a urdf is passed through), and it guards the planner,
                               not isaac - a direct move_to_joint_positions can still
                               wind a joint up.
  joint_limits_deg (object)  - per-joint [min, max] in degrees, e.g.
                               {"shoulder_lift_joint": [-270, 90]}. Overrides
                               joint_limit_deg for the joints it names. Use it when a
                               joint's working range straddles the symmetric limit: a
                               span of 360 deg still admits only one solution per pose,
                               but the ends can be put where the arm never travels.
  kinematics_url (string)    - where to fetch the kinematics file served by
                               GetKinematics (.json = SVA, .urdf = URDF;
                               file:// URLs work). Known assets with official
                               viam kinematics (ur3e/ur5e/ur20) fetch them
                               automatically.
"""

import asyncio
import hashlib
import json
import math
import os
import tempfile
import time
import urllib.request
from typing import Any, ClassVar, Dict, List, Mapping, Optional, Sequence, Tuple

from typing_extensions import Self
from viam.logging import getLogger
from viam.components.arm import Arm, JointPositions, KinematicsFileFormat, Pose
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import Geometry, ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes

from .. import FAMILY, NAMESPACE
from ..sim_manager import KNOWN_ASSETS, ArmHandle, SimManager
from ..spatial import quat_to_ov
from .utils import apply_frame_to_attrs, get_attrs, validate_sim_component

LOGGER = getLogger("viam-isaac-sim.arm")

_TOLERANCE_RAD = math.radians(0.5)


class IsaacArm(Arm, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily(NAMESPACE, FAMILY), "arm")

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._handle: Optional[ArmHandle] = None
        self._attrs: Dict[str, Any] = {}
        self._move_timeout = 30.0
        self._speed_deg_s = 60.0
        self._accel_deg_s2 = 120.0
        self._kinematics: Optional[Tuple[KinematicsFileFormat.ValueType, bytes]] = None
        # Which leg of the round is being taught right now, and what has been taught.
        self._recording: Optional[str] = None
        self._taught: Dict[str, List[List[float]]] = {}

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        arm = cls(config.name)
        arm.reconfigure(config, dependencies)
        return arm

    @classmethod
    def validate_config(
        cls, config: ComponentConfig
    ) -> Tuple[Sequence[str], Sequence[str]]:
        return validate_sim_component(config)

    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> None:
        attrs = apply_frame_to_attrs(config, get_attrs(config))
        self._move_timeout = float(attrs.get("move_timeout_sec", 30.0))
        self._speed_deg_s = float(attrs.get("joint_speed_deg_s", 60.0))
        self._accel_deg_s2 = float(attrs.get("joint_accel_deg_s2", 120.0))
        # Drop the cached kinematics: joint_limit_deg and joint_limits_deg are clamped
        # into the file this serves, so a cache that survives a reconfigure keeps handing
        # the planner the limits the cell used to have. Measured, reverting the wrist range
        # in the fragment left the arm still advertising the wider one, and the planner
        # went on choosing solutions the config had just ruled out.
        if self._attrs.get("joint_limit_deg") != attrs.get("joint_limit_deg") or \
                self._attrs.get("joint_limits_deg") != attrs.get("joint_limits_deg") or \
                self._attrs.get("kinematics_url") != attrs.get("kinematics_url"):
            self._kinematics = None
        self._attrs = attrs
        self._handle = SimManager.get().create_arm(self.name, attrs)

    def _h(self) -> ArmHandle:
        if self._handle is None:
            raise RuntimeError(f"arm {self.name} is not attached to the sim")
        return self._handle

    async def get_end_position(self, **kwargs) -> Pose:
        (x, y, z), quat = await asyncio.to_thread(self._h().get_end_pose)
        ox, oy, oz, theta = quat_to_ov(quat)
        return Pose(
            x=x * 1000.0,
            y=y * 1000.0,
            z=z * 1000.0,
            o_x=ox,
            o_y=oy,
            o_z=oz,
            theta=math.degrees(theta),
        )

    async def move_to_position(self, pose: Pose, **kwargs) -> None:
        raise NotImplementedError(
            "IK and motion planning are Viam's job, not the module's: use the "
            "motion service (needs GetKinematics, on the roadmap) or "
            "move_to_joint_positions"
        )

    # A joint still closing on its target is not stuck, however long it takes. Give up
    # this long after the arm last got measurably nearer, rather than after a fixed
    # wall-clock budget, which cannot tell a slow move from a blocked one. A blocked arm
    # also fails sooner this way than it did on the old thirty second budget.
    # Long enough that an arm closing the last degree or two slowly is allowed to
    # finish. Six seconds was not: the round's big joint-space moves are walked in five
    # degree sub-targets, the arm lags a degree or two behind each one, and the final
    # sub-target has to be met to within half a degree - so a move that had travelled
    # 200 degrees correctly failed on the last two, about one round in three.
    _STALL_AFTER = 15.0
    # How much nearer counts as nearer, so sensor noise cannot pass for progress.
    _PROGRESS_RAD = math.radians(0.25)

    # Inside this much of the target, a drive that is still turning is converging, not
    # stalled, and the stall detector has to leave it alone.
    #
    # A critically damped drive closes the last fraction of a degree asymptotically, so
    # near the target it improves by less than _PROGRESS_RAD per _STALL_AFTER window and
    # the detector called it stalled. Measured, that is what the presentations were
    # failing on: "it stopped a short way short with effort to spare ... j3: at -22.8 want
    # -22.2 ... moving=True speeds_deg_s=[..., 17.6]". Six tenths of a degree, with the
    # arm still moving, cost the round a whole face - and three attempts at 19 s each.
    #
    # Progress is the right test for a jam far from the target. It is the wrong test for
    # the last half-degree, where the honest signal is whether the joints are still moving
    # at all.
    _CONVERGING_RAD = math.radians(3.0)

    # Moves are timed in SIM seconds. Timed on the wall, a trajectory ran ahead of the
    # physics whenever the sim fell behind real time - four learner cameras streaming took
    # it to 0.21x, and arm-a's first move "did not settle" without having moved
    # (2026-10-14) - and a cycle measured in sim seconds came out shorter the slower the
    # sim ran. A wall limit still stands behind it, for a sim that is paused.
    _WALL_FACTOR = 20.0

    @staticmethod
    def _clock() -> float:
        sim = SimManager.get()
        if sim.mock or not sim._booted.is_set():
            return time.monotonic()        # no sim stepping: the wall is the only clock
        return float(sim._sim_clock)

    async def _settle(
        self, handle, targets: Sequence[float], tolerance: float, cap: float
    ) -> Optional[List[float]]:
        """Wait for the joints to arrive. Returns None, or where they got stuck.

        Progress, not elapsed time, is what says whether the arm is moving. An earlier
        version waited a fixed ten seconds per intermediate waypoint and thirty for the
        final one, so a move that was merely slow and a joint that was truly jammed
        produced the same message and the same joint list.
        """
        best: Optional[float] = None
        improved = self._clock()
        deadline = improved + cap
        wall_deadline = time.monotonic() + cap * self._WALL_FACTOR
        current: List[float] = []
        while self._clock() < deadline and time.monotonic() < wall_deadline:
            current = await asyncio.to_thread(handle.get_joint_positions)
            if len(current) >= len(targets) and all(
                abs(c - t) <= tolerance for c, t in zip(current, targets)
            ):
                return None
            if current:
                error = max(abs(c - t) for c, t in zip(current, targets))
                if best is None or error < best - self._PROGRESS_RAD:
                    best, improved = error, self._clock()
                elif self._clock() - improved > self._STALL_AFTER:
                    # Still closing the last few degrees? Then it is converging, and the
                    # deadline is the only thing entitled to stop it.
                    if error <= self._CONVERGING_RAD and await asyncio.to_thread(
                            handle.is_moving):
                        improved = self._clock()
                    else:
                        return current
            await asyncio.sleep(0.02)
        return current

    @staticmethod
    def _stuck(current: Sequence[float], targets: Sequence[float],
               tolerance: float) -> str:
        return ", ".join(
            f"j{j}: at {math.degrees(c):.1f} want {math.degrees(t):.1f}"
            for j, (c, t) in enumerate(zip(current, targets))
            if abs(c - t) > tolerance
        )

    async def _velocities(self, handle) -> str:
        """Whether a stalled arm is pressing on something or still creeping.

        The two look identical in a joint-angle report and want opposite fixes: a
        blocked arm needs the obstacle moving, a slow one needs longer to arrive. Every
        stall in this cell has been diagnosed by guessing which; the velocity says.
        """
        try:
            moving = await asyncio.to_thread(handle.is_moving)
            art = getattr(handle, "_art", None)
            if art is None:
                return f"moving={moving}"
            speeds = await asyncio.to_thread(
                lambda: [float(v) for v in art.get_joint_velocities()])
            return (f"moving={moving} speeds_deg_s="
                    f"{[round(math.degrees(v), 2) for v in speeds]}")
        except Exception:  # noqa: BLE001
            return "velocities unavailable"

    async def move_to_joint_positions(
        self, positions: JointPositions, **kwargs
    ) -> None:
        """Go to one joint configuration, tracked.

        The same executor as a trajectory, and for the same reason. This used to command
        the target and wait, which is fine for a nudge and wrong for a move: sent
        straight to a configuration a long way off, the drive saturates, the arm slams
        at it, overshoots and hunts. It went unnoticed while the only callers were the
        small ones - the shoulder lift off the belt, the wrist unwind - and surfaced the
        moment home became a joint move, as `arm-b stopped short of its target` on every
        round.
        """
        await self._replay([list(positions.values)],
                           speed_deg_s=self._speed_deg_s, settle=True)

    # ---- teaching and replay --------------------------------------------------
    #
    # A cell like this runs one fixed sequence over and over, and asking the motion
    # service to plan it again every cycle is asking a randomised algorithm for a
    # different answer each time. It duly gives one: the planner seeds IK from wherever
    # the arm happens to be and scores by joint distance, so the same station is reached
    # through a different arm configuration on different runs - which is what "the
    # movements are wonky" was, and what left the marginal poses failing on some cycles
    # and not others.
    #
    # So plan each leg once, keep the joint path it produced, and replay that. This is
    # what an industrial cell does: the program is taught, not re-derived per part.

    def _start_recording(self, label: str, reset: bool = False) -> None:
        self._recording = label
        if reset:
            self._taught[label] = []
        else:
            self._taught.setdefault(label, [])

    def _stop_recording(self) -> None:
        self._recording = None

    async def _replay(self, path_deg: Sequence[Sequence[float]],
                      speed_deg_s: float = 60.0, settle: bool = True) -> None:
        """Walk a taught joint path at a controlled rate.

        Not a sequence of move-and-wait hops. Each hop's drive target was a long way
        off, so the arm accelerated hard, overshot and hunted - measured, it sat
        `moving=True` at up to 2.7 deg/s a degree or two short of target and never
        settled. Here the target is only ever a step ahead of where the arm is, so the
        drive is never chasing a large error and the arm tracks the path instead of
        lunging at it.

        And the rate RAMPS, over the whole path rather than per segment.

        It used to step straight to the full rate on the first tick and stop dead on the
        last. Computed from a taught program at 40 deg/s: within every move the commanded
        rate was a flat 38-40 deg/s and no waypoint changed it by more than a few deg/s -
        the path itself was smooth - but each of the round's 50 moves began with a 0 to
        40 deg/s step in one 20 ms tick and ended with the reverse. That is 100
        instantaneous velocity steps a round, and with 2.5 kg on a 206 mm lever it is
        visible: the arm and the part shake at the start and end of every move, and worst
        on the long ones where there is most to stop.

        So the path is walked against a trapezoidal profile in its own arc length: ramp up
        at `accel_deg_s2`, cruise at `speed_deg_s`, ramp down to rest. A short path never
        reaches the cruise rate and is a pure ramp up and down, which is most of this
        cell's moves.
        """
        handle = self._h()
        here = list(await asyncio.to_thread(handle.get_joint_positions))
        path = [here] + [[math.radians(v) for v in w] for w in path_deg]
        tick = 0.02
        speed = math.radians(speed_deg_s)
        accel = math.radians(self._accel_deg_s2)

        # Arc length along the path, measured by the joint that moves furthest in each
        # segment - the same measure the rate is expressed in.
        spans = [max((abs(b - a) for a, b in zip(start, goal)), default=0.0)
                 for start, goal in zip(path, path[1:])]
        marks, run = [0.0], 0.0
        for span in spans:
            run += span
            marks.append(run)
        total = marks[-1]
        if total <= 0.0:
            await asyncio.to_thread(handle.set_joint_targets, list(path[-1]))
        else:
            def at(distance: float):
                """The configuration this far along the path."""
                for index, span in enumerate(spans):
                    if span <= 0.0:
                        continue
                    if distance <= marks[index + 1] or index == len(spans) - 1:
                        fraction = min(1.0, max(0.0, (distance - marks[index]) / span))
                        return [a + (b - a) * fraction
                                for a, b in zip(path[index], path[index + 1])]
                return list(path[-1])

            # Trapezoid, clipped to a triangle when the path is too short to reach speed.
            ramp = speed * speed / accel
            if total < ramp:
                peak = math.sqrt(total * accel)
                accel_t = peak / accel
                duration, cruise_t = 2.0 * accel_t, 0.0
            else:
                peak = speed
                accel_t = speed / accel
                cruise_t = (total - ramp) / speed
                duration = 2.0 * accel_t + cruise_t

            elapsed = 0.0
            last = self._clock()
            wall_deadline = time.monotonic() + (duration + 5.0) * self._WALL_FACTOR
            while elapsed < duration and time.monotonic() < wall_deadline:
                await asyncio.sleep(tick)
                now = self._clock()
                # Along the profile by the SIM time that has passed, never more than a few
                # ticks at once (a reset rewinds nothing here: the clock never rewinds).
                elapsed = min(duration, elapsed + min(max(0.0, now - last), 5 * tick))
                last = now
                if elapsed <= accel_t:
                    travelled = 0.5 * accel * elapsed * elapsed
                elif elapsed <= accel_t + cruise_t:
                    travelled = 0.5 * peak * accel_t + peak * (elapsed - accel_t)
                else:
                    left = duration - elapsed
                    travelled = total - 0.5 * accel * left * left
                await asyncio.to_thread(handle.set_joint_targets, at(travelled))
            await asyncio.to_thread(handle.set_joint_targets, list(path[-1]))
        if not settle:
            return
        stuck = await self._settle(handle, path[-1], _TOLERANCE_RAD,
                                   cap=self._move_timeout)
        if stuck is None:
            return
        # A standing error with effort to spare IS arrival, for a drive like this one.
        #
        # A position-controlled drive settles where stiffness times error equals the load
        # and cannot deflect further - the module says so in `_set_drive_gains` - so a
        # joint a fraction of a degree short with its effort nowhere near its limit has
        # finished moving. Calling that a failure is a category error, and an expensive
        # one: it failed a whole teaching pass at the set-down with j3 0.7 degrees off
        # target, creeping, and 28 N.m of a 1000 N.m limit.
        #
        # The two cases this does NOT cover still raise, which is the point: a joint far
        # from target has not arrived, and a joint pinned at its limit cannot.
        error = max(abs(c - t) for c, t in zip(stuck, path[-1]))
        efforts_now = await asyncio.to_thread(handle.get_efforts)
        measured = efforts_now.get("measured") or []
        limits = efforts_now.get("limits") or []
        pinned = any(cap and abs(m) >= 0.98 * abs(cap)
                     for m, cap in zip(measured, limits))
        if error <= self._CONVERGING_RAD and not pinned:
            LOGGER.info(
                "arm %s settled %.2f degrees short of target with effort to spare, "
                "which is where its drives hold; treating that as arrived",
                self.name, math.degrees(error))
            return
        speeds = await self._velocities(handle)
        # What the drives are doing, because "did not settle" has two causes that look
        # identical in joint angles and are opposite in what to do about them. A drive
        # short of its target with its effort AT the limit is out-torqued: the arm cannot
        # hold this pose and the gains or the pose have to change. One short of target
        # with effort well UNDER the limit is being pushed - something is in contact -
        # and raising gains would only push back harder. Measured on this cell, holding
        # the part over the handoff stand takes 37 N m of a 150 N m shoulder, so a stall
        # there is the second kind, and every previous stall here was treated as the
        # first.
        efforts = await asyncio.to_thread(handle.get_efforts)
        await asyncio.to_thread(handle.stop)
        raise TimeoutError(
            f"arm {self.name} did not settle at the end of a taught path - "
            f"{self._why(stuck, path[-1], efforts)} ({speeds}; "
            f"joints off target: {self._stuck(stuck, path[-1], _TOLERANCE_RAD)}; "
            f"measured effort {efforts.get('measured')} of {efforts.get('limits')})")

    # How far off target stops being a drive's standing error and starts being a joint
    # that never arrived.
    _ADRIFT_RAD = math.radians(30.0)

    def _why(self, current: Sequence[float], targets: Sequence[float],
             efforts: Dict[str, Any]) -> str:
        """Name the condition, because "stuck" was the wrong word for all of them.

        Measured on this cell, the message that said "stuck joints" was reporting an arm
        travelling at 38 deg/s with one joint 196 degrees from target and pan's effort
        pinned at its limit - not stuck, but unable to keep up with the path it was given,
        because the path asked for a whole-arm reconfiguration and the dynamic load
        saturated the base. That reads completely differently from a joint held a
        fraction short by gravity, and the two want different fixes: re-plan the move
        versus accept the standing error.
        """
        measured = efforts.get("measured") or []
        limits = efforts.get("limits") or []
        pinned = [f"j{j}" for j, (m, cap) in enumerate(zip(measured, limits))
                  if cap and abs(m) >= 0.98 * abs(cap)]
        adrift = [f"j{j}" for j, (c, t) in enumerate(zip(current, targets))
                  if abs(c - t) > self._ADRIFT_RAD]
        if adrift and pinned:
            return (f"it could not keep up with the path: {', '.join(adrift)} never "
                    f"arrived and {', '.join(pinned)} is at its effort limit, so the "
                    f"move asks for more than the drives can deliver - re-plan it")
        if adrift:
            return (f"{', '.join(adrift)} never arrived, with no drive at its limit, so "
                    f"something stopped the arm rather than out-torqued it")
        if pinned:
            return (f"{', '.join(pinned)} is at its effort limit a short way short, "
                    f"which is a drive holding a pose it cannot quite reach")
        return "it stopped a short way short with effort to spare"

    async def move_through_joint_positions(
        self, positions: Sequence[JointPositions], *args, **kwargs
    ) -> None:
        """Execute a trajectory - what the motion service calls to run a planned path.

        Executed the same way a taught path is: tracked, not stepped. The previous
        version drove to each waypoint and waited for the arm to settle within half a
        degree before issuing the next, which meant the drive was always chasing a
        target a long way off, and at the extended poses the arm overshot and hunted -
        measured, sitting `moving=True` at up to 2.7 deg/s a degree or two short,
        indefinitely. About half of all rounds lost a move to it.

        Tracking a path costs nothing extra and removes the whole failure mode: the
        commanded target is never more than a tick ahead of where the arm is, so there
        is no large error to overshoot on, and only the final pose has to settle.
        """
        waypoints = list(positions)
        if not waypoints:
            return
        if self._recording:
            self._taught.setdefault(self._recording, []).extend(
                [list(w.values) for w in waypoints])
        await self._replay([list(w.values) for w in waypoints],
                           speed_deg_s=self._speed_deg_s, settle=True)

    async def get_joint_positions(self, **kwargs) -> JointPositions:
        radians = await asyncio.to_thread(self._h().get_joint_positions)
        return JointPositions(values=[math.degrees(r) for r in radians])

    async def stop(self, **kwargs) -> None:
        await asyncio.to_thread(self._h().stop)

    async def is_moving(self) -> bool:
        return await asyncio.to_thread(self._h().is_moving)

    # viam-sdk 0.82 made these five abstract (see requirements.txt). A simulated arm has
    # no meshes of its own to serve, no manual (freedrive) mode and no streamed
    # trajectory input, so each says so; their types are imported here, not at the top,
    # because 0.80 does not have them and the module must still load there.
    async def get_3d_models(self, **kwargs) -> Mapping[str, Any]:
        return {}

    async def get_manual_mode(self, **kwargs) -> bool:
        return False

    async def set_manual_mode(self, manual_mode: bool, enabled_for: int = 0, **kwargs) -> None:
        raise NotImplementedError("a simulated arm has no manual mode")

    async def get_properties(self, **kwargs):
        from viam.proto.component.arm import GetPropertiesResponse

        return GetPropertiesResponse()

    async def move_through_joint_positions_streamed(self, batches, **kwargs):
        raise NotImplementedError("streamed trajectories are not supported; use "
                                  "move_through_joint_positions")

    def _kinematics_url(self) -> Optional[str]:
        url = self._attrs.get("kinematics_url")
        if url:
            return str(url)
        asset = self._attrs.get("asset")
        if asset and asset in KNOWN_ASSETS:
            return KNOWN_ASSETS[asset].get("kinematics")
        return None

    def _load_kinematics(self) -> Tuple[KinematicsFileFormat.ValueType, bytes]:
        url = self._kinematics_url()
        if not url:
            raise NotImplementedError(
                f"no kinematics file known for arm {self.name}; set the "
                '"kinematics_url" attribute (SVA .json or .urdf)'
            )
        ext = os.path.splitext(url)[1].lower()
        fmt = (
            KinematicsFileFormat.KINEMATICS_FILE_FORMAT_URDF
            if ext in (".urdf", ".xml", ".xacro")
            else KinematicsFileFormat.KINEMATICS_FILE_FORMAT_SVA
        )

        cache_dir = os.environ.get("VIAM_MODULE_DATA") or tempfile.gettempdir()
        cache = os.path.join(
            cache_dir,
            f"kinematics-{hashlib.sha1(url.encode()).hexdigest()[:12]}{ext}",
        )
        if os.path.exists(cache):
            with open(cache, "rb") as f:
                return fmt, f.read()

        self.logger.info("fetching kinematics for %s from %s", self.name, url)
        with urllib.request.urlopen(url, timeout=30) as resp:
            data = resp.read()
        try:
            os.makedirs(cache_dir, exist_ok=True)
            tmp = cache + ".tmp"
            with open(tmp, "wb") as f:
                f.write(data)
            os.replace(tmp, cache)
        except OSError:
            pass  # caching is best-effort
        return fmt, data

    def _clamp_joint_limits(
        self, fmt: KinematicsFileFormat.ValueType, data: bytes
    ) -> bytes:
        """Narrow the joint ranges the motion service is told about.

        The UR SVA allows every revolute joint +-360 degrees, which is true of the real
        robot. Viam's planner may pick any solution inside that range and has no reason to
        prefer the one nearest the arm's current pose, so a two-arm cell winds itself up:
        measured over one pass of this cell's stations, a joint went from -26 to 206
        degrees, and after a second pass an arm sat at 270. Approaching 360 the cell fails
        three different-looking ways - the planner cannot find a route, the arm cannot
        settle and times out, or the move is rejected as out of range - which is why the
        symptom looked intermittent.

        Clamping costs no reachable pose: +-180 degrees of a revolute joint already covers
        every orientation. Surveyed across this cell's stations the solutions needed at
        most [162, 174, 104, 157, 90] degrees on the first five joints, and 302 on wrist_3
        - which is the wind-up itself, 302 being the same place as -58.

        This is a guard on what the PLANNER believes, not something isaac enforces:
        set_joint_targets will still drive a joint anywhere it is told, so a direct
        move_to_joint_positions can still wind up. Sending the arms to a known
        configuration between rounds is what covers that path.

        A symmetric ±limit is not always the right shape. Clamping puts the wrap point
        at the limit on *every* joint, and a joint whose working range straddles that
        just ratchets there instead - measured here, bounding pan at ±180 moved the
        problem from pan to shoulder-lift, which then sat pinned at 180. `joint_limits_deg`
        gives one joint an explicit [min, max] so the discontinuity can be placed
        somewhere the arm never goes: this cell's lift works in about [-174, 4], so
        [-270, 90] spans a single turn while keeping both ends far from the traffic.

        Only SVA json is rewritten. A urdf is passed through untouched.
        """
        limit = self._attrs.get("joint_limit_deg")
        per_joint = self._attrs.get("joint_limits_deg") or {}
        if limit is None and not per_joint:
            return data
        if fmt != KinematicsFileFormat.KINEMATICS_FILE_FORMAT_SVA:
            self.logger.warning(
                "%s: joint limits are only applied to SVA kinematics; this arm serves "
                "a urdf, so they are being ignored", self.name,
            )
            return data
        limit = abs(float(limit)) if limit is not None else None

        model = json.loads(data)
        narrowed = []
        for joint in model.get("joints", []):
            if joint.get("type") != "revolute":
                continue
            joint_id = joint.get("id")
            before = (joint.get("min"), joint.get("max"))
            override = per_joint.get(joint_id)
            if override is not None:
                low, high = (float(v) for v in override)
            elif limit is not None:
                low, high = -limit, limit
            else:
                continue
            joint["min"] = max(float(joint.get("min", low)), low)
            joint["max"] = min(float(joint.get("max", high)), high)
            if (joint["min"], joint["max"]) != before:
                narrowed.append(f"{joint_id} {before} -> "
                                f"({joint['min']}, {joint['max']})")
        if narrowed:
            self.logger.info("%s: narrowed joint limits: %s",
                             self.name, "; ".join(narrowed))
        return json.dumps(model).encode()

    async def get_kinematics(self, **kwargs) -> Tuple[KinematicsFileFormat.ValueType, bytes]:
        if self._kinematics is None:
            fmt, data = await asyncio.to_thread(self._load_kinematics)
            # Clamp here rather than in _load_kinematics: that caches the fetched file on
            # disk, and the cache must hold what upstream served, not this arm's config.
            self._kinematics = (fmt, self._clamp_joint_limits(fmt, data))
        return self._kinematics

    async def get_geometries(self, **kwargs) -> List[Geometry]:
        return []

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Mapping[str, ValueTypes]:
        name = command.get("command")
        if name == "get_joint_positions_radians":
            return {"values": await asyncio.to_thread(self._h().get_joint_positions)}
        if name == "record":
            self._start_recording(str(command.get("label", "")),
                                  reset=bool(command.get("reset")))
            return {"recording": self._recording}
        if name == "efforts":
            return await asyncio.to_thread(self._h().get_efforts)
        if name == "close_recording":
            # Close the label without reading the program back. A label left open keeps
            # collecting: an idle arm is held at its current configuration through the
            # trajectory entry point, so every hold taken while the OTHER arm works
            # lands in whichever label was open last. Measured, that turned arm-a's
            # 18-waypoint retreat into 66 waypoints, with the arm's later trip home
            # recorded in the middle of it as a 217 degree stride the teaching pass
            # never made in one go. A label is open only while the move it names runs.
            closed = self._recording
            self._stop_recording()
            return {"closed": closed,
                    "path": list(self._taught.get(closed or "", []))}
        if name == "stop_recording":
            self._stop_recording()
            return {"taught": {k: v for k, v in self._taught.items()}}
        if name == "taught":
            return {"taught": {k: v for k, v in self._taught.items()}}
        if name == "forget":
            self._taught.clear()
            return {"taught": {}}
        if name == "replay":
            path = command.get("path")
            if not path:
                raise ValueError("replay needs a 'path' of joint positions in degrees")
            # The arm's OWN configured speed unless the caller insists. The default used
            # to be 60 while the fragment configured 30, so every replay ran at twice the
            # speed the cell was tuned for - and a position drive's damping resists motion
            # at d*omega, so 60 deg/s against damping 300 demands 314 N.m of a joint
            # allowed 150. Replays were saturating by a factor of two and the number the
            # fragment sets had nothing to do with it.
            await self._replay(
                [[float(v) for v in w] for w in path],
                speed_deg_s=float(command.get("speed_deg_s") or self._speed_deg_s),
                settle=bool(command.get("settle", True)))
            return {"replayed": len(path)}
        raise ValueError(f"unknown command: {command}")
