"""The singleton that owns Isaac Sim.

Isaac Sim (Omniverse Kit) wants to be created and stepped from a single
thread, so the module runs it on the process main thread (see main.py) and
everything else - the Viam module server, component handlers - submits work
to that thread through a queue. Handles returned by create_arm/create_camera/
create_base wrap that queue so component models can stay simple.

A "mock" backend (world attribute: {"mock": true}) implements the same
handle interfaces with plain python so the module can run and be tested on
machines without Isaac Sim installed.
"""

import math
import queue
import sys
import threading
import time
from concurrent.futures import Future
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from viam.logging import getLogger

LOGGER = getLogger("viam-isaac-sim")


def _as_radians(degrees):
    import numpy as np

    return np.array([math.radians(float(v)) for v in degrees], dtype=float)


def _as_list(values) -> Any:
    """Gains come back as numpy arrays, which log as unreadable reprs."""
    if values is None:
        return None
    try:
        return [round(float(v), 1) for v in values]
    except TypeError:
        return values

# Assets shipped on the Isaac Sim nucleus/content server, addressable by a
# short name in component config. Paths are relative to the assets root;
# where isaac 5.0 moved an asset, the 5.0 path is listed first with the 4.x
# path as a fallback - the first candidate that exists is used.
_UR_KINEMATICS = "https://raw.githubusercontent.com/viam-modules/universal-robots/main/src/kinematics"

# Isaac's Universal Robots assets are URDF imports, so their root prim is ROS
# `base_link`. Viam's UR kinematics are expressed in the UR *controller's* `base`
# frame, and the two differ by a half turn about Z. Spawning the USD unrotated
# therefore mirrors every x and y between what the motion service plans and what
# Isaac simulates - the planner returns a clear path and the arm drives elsewhere.
#
# Measured on ur5e, all joints zero, USD spawned at the origin with identity
# orientation: Isaac puts the flange at (817.2, 232.9, 63.1) mm while the SVA chain
# gives (-817.2, -232.9, 62.8). Composing this rotation into the spawn lines them up.
_UR_BASE_ROTATION_WXYZ = (0.0, 0.0, 0.0, 1.0)  # 180 deg about Z

KNOWN_ASSETS: Dict[str, Dict[str, Any]] = {
    "ur3e": {
        "usd": ["/Isaac/Robots/UniversalRobots/ur3e/ur3e.usd"],
        "kinematics": f"{_UR_KINEMATICS}/ur3e.json",
        "base_rotation_wxyz": _UR_BASE_ROTATION_WXYZ,
    },
    "ur5e": {
        "usd": ["/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"],
        "kinematics": f"{_UR_KINEMATICS}/ur5e.json",
        "base_rotation_wxyz": _UR_BASE_ROTATION_WXYZ,
    },
    "ur10": {
        "usd": ["/Isaac/Robots/UniversalRobots/ur10/ur10.usd"],
        "base_rotation_wxyz": _UR_BASE_ROTATION_WXYZ,
    },
    "ur10e": {
        "usd": ["/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd"],
        "base_rotation_wxyz": _UR_BASE_ROTATION_WXYZ,
    },
    "ur16e": {
        "usd": ["/Isaac/Robots/UniversalRobots/ur16e/ur16e.usd"],
        "base_rotation_wxyz": _UR_BASE_ROTATION_WXYZ,
    },
    "ur20": {
        "usd": ["/Isaac/Robots/UniversalRobots/ur20/ur20.usd"],
        "kinematics": f"{_UR_KINEMATICS}/ur20.json",
        "base_rotation_wxyz": _UR_BASE_ROTATION_WXYZ,
    },
    # Isaac's uf850 is authored in viam's SVA base frame: at all-zero joints link6 sits at
    # (149.999, -0.004, 238.000) mm rotated a half turn about X, which is the xarm850 SVA
    # chain's end-effector pose to 0.005 mm - so no base rotation, and link6 IS viam's
    # end-effector frame (tool along local +Z). Its gripper variant defaults to "None",
    # so the articulation is the six arm joints.
    "uf850": {
        "usd": ["/Isaac/Robots/Ufactory/uf850/uf850.usd"],
        "kinematics": "https://raw.githubusercontent.com/viam-modules/viam-ufactory-xarm/"
                      "main/arm/xarm850_kinematics.json",
    },
    "franka": {
        "usd": [
            "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
            "/Isaac/Robots/Franka/franka.usd",
        ]
    },
    "jetbot": {
        "usd": [
            "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd",
            "/Isaac/Robots/Jetbot/jetbot.usd",
        ],
        "wheel_joints": ["left_wheel_joint", "right_wheel_joint"],
        "wheel_radius": 0.03,
        "wheel_base": 0.1125,
    },
}


GROUND_PLANE_NAME = "ground_plane"
GROUND_PLANE_PRIM_PATH = "/World/GroundPlane"
DOME_LIGHT_PRIM_PATH = "/World/DomeLight"
# Where joints the CELL makes live, kept apart from any the assets author themselves.
JOINTS_PRIM_PATH = "/World/CellJoints"
SPHERE_LIGHT_PRIM_PATH = "/World/defaultLight"
MATTE_OBJECT_SETTING = "/rtx/matteObject/enabled"
SHADOW_CATCHER_SETTING = "/rtx/shadowCatcher/enabled"
VIEWPORT_GRID_SETTING = "/app/viewport/grid/enabled"
DEFAULT_DOME_INTENSITY = 1000.0
# Near and far planes, metres. The near plane has to be closer than anything the camera
# is meant to see; usd's own default of 1.0 is further than an inspection station.
DEFAULT_CLIPPING_RANGE_M = (0.02, 1000.0)
DEFAULT_DOME_COLOR = (1.0, 1.0, 1.0)

# A floor, not a drawing of one. Isaac's default is a lit grid, which reads as a
# CAD viewport rather than a room - the thing that made the first cell video look
# like parts floating over graph paper.
GROUND_DEFAULTS: Dict[str, Any] = {
    "kind": "grid",
    "size": 40.0,
    "color": (0.28, 0.28, 0.30),
    "friction": 0.6,
    "restitution": 0.0,
    "matte": False,
}


def ground_plan(
    ground: Optional[Dict[str, Any]], usd_stage: Optional[str]
) -> Tuple[str, Dict[str, Any]]:
    """Pure. What `_boot` should author for the floor.

    "skip" when a ground config is set alongside someone's own usd_stage, "grid" for the
    default, "none" to author nothing, or "plane" with the kwargs add_ground_plane takes.
    """
    if usd_stage is not None and ground is not None:
        return "skip", {"reason": f"usd_stage {usd_stage!r} is set"}
    if ground is None:
        return "grid", {}
    kind = ground.get("kind", GROUND_DEFAULTS["kind"])
    if kind == "grid":
        return "grid", {}
    if kind == "none":
        return "none", {}
    friction = float(ground.get("friction", GROUND_DEFAULTS["friction"]))
    return "plane", {
        "size": float(ground.get("size", GROUND_DEFAULTS["size"])),
        "color": [float(v) for v in ground.get("color", GROUND_DEFAULTS["color"])],
        "static_friction": friction,
        "dynamic_friction": friction,
        "restitution": float(ground.get("restitution", GROUND_DEFAULTS["restitution"])),
    }


def ground_is_matte(ground: Optional[Dict[str, Any]]) -> bool:
    """Pure. A matte floor is invisible to primary rays but still catches shadows.

    Useful for compositing a cell over a backdrop; wrong for a room, where the floor is
    part of what you are looking at. Off by default for that reason.
    """
    if ground is None:
        return bool(GROUND_DEFAULTS["matte"])
    return bool(ground.get("matte", GROUND_DEFAULTS["matte"]))


@dataclass
class SimConfig:
    mock: bool = False
    headless: bool = True
    livestream: bool = True
    usd_stage: Optional[str] = None
    physics_dt: float = 1.0 / 60.0
    rendering_dt: float = 1.0 / 60.0
    # The fastest the sim may run against the wall clock; 0 for as fast as it can. A
    # learner's cell runs in real time (1.0), so what they see an arm do takes as long
    # as it would on a real cell.
    max_rtf: float = 0.0
    boot_timeout: float = 300.0
    # IP the livestream advertises to clients; auto-detected if empty
    livestream_public_ip: str = ""
    # resolution kit renders (and therefore streams) at
    livestream_width: int = 1280
    livestream_height: int = 720
    # props to spawn into the scene at boot; each entry:
    #   {"type": "cube"|"usd", "name": ..., "position": [x,y,z] (m),
    #    "size": edge_m, "scale": [sx,sy,sz], "color": [r,g,b] 0-1,
    #    "fixed": bool, "usd_path": ...}
    props: List[Dict[str, Any]] = field(default_factory=list)
    # kit console verbosity (verbose/info/warning/error). Kit prints thousands
    # of lines at info, and viam-server records the module's stderr as
    # error-level logs, so default to warning.
    kit_log_level: str = "warning"
    # scene presentation. `ground` picks what the floor is, `render` carries the
    # viewport overlays, `lighting` the dome and key lights.
    ground: Optional[Dict[str, Any]] = None
    render: Dict[str, Any] = field(default_factory=dict)
    lighting: Dict[str, Any] = field(default_factory=dict)


class SimManager:
    """Owns the sim thread. Get the process-wide instance via SimManager.get()."""

    _instance: Optional["SimManager"] = None
    _instance_lock = threading.Lock()

    @classmethod
    def get(cls) -> "SimManager":
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = SimManager()
            return cls._instance

    def __init__(self) -> None:
        self._tasks: "queue.Queue[Tuple[Callable[[], Any], Future]]" = queue.Queue()
        self._boot_requested = threading.Event()
        self._booted = threading.Event()
        self._boot_error: Optional[BaseException] = None
        self._stop = threading.Event()
        self._sim_thread_id: Optional[int] = None

        self.cfg: Optional[SimConfig] = None
        self.mock = False
        self._sim_app = None
        self.world = None
        self._isaac = None  # lazily-populated namespace of isaac imports
        self._step_callbacks: Dict[str, Callable[[float], None]] = {}
        # component name -> (spawn attrs, handle). viam-server rebuilds
        # resources on config change, but prims can't be re-spawned without
        # restarting kit, so handles are cached per component name.
        self._handles: Dict[str, Tuple[Dict[str, Any], Any]] = {}
        # Per arm: what drive_max_effort asked for and whether anything accepted it.
        self._gain_notes: Dict[str, Dict[str, Any]] = {}
        self._drive_force_notes: Dict[str, Dict[str, Any]] = {}
        # What `_make_static` found and froze, per prop. Logs on this machine go
        # nowhere, and "it stopped falling" proves bodies existed without saying WHICH
        # prims carry them - which is exactly what a conveyor drive needs to know.
        self._static_notes: Dict[str, List[str]] = {}
        # Conveyors, by prop name. Each holds the driven prims, the geometry the beam
        # is cast through, and whether it is currently running.
        self._conveyors: Dict[str, Dict[str, Any]] = {}
        # Where the sim thread's wall time goes, cumulative: draining submitted work vs
        # stepping. Read by `status` and `profile`, so a slow run says why.
        self._loop_s: Dict[str, float] = {"drain": 0.0, "step": 0.0, "n": 0.0}
        # Sim seconds since boot that never rewind: a `reset` puts the world's own clock
        # back to zero, and a cycle that restages a shell midway is still one cycle.
        self._sim_clock = 0.0
        # The belt index in progress (see `index`), decided per physics step.
        self._index_state: Optional[Dict[str, Any]] = None
        self._index_coast_s: Optional[float] = None   # learned belt coast, s per mm/s at the cut
        self._index_cb: set = set()
        self._camera_handles: List[Any] = []

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------

    def ensure_booted(self, cfg: SimConfig) -> None:
        """Called by the world component's reconfigure. Boots the sim on the
        sim thread the first time; subsequent calls with a different config
        log that a module restart is required (Kit can't be re-created)."""
        if self._booted.is_set():
            if self.cfg != cfg:
                LOGGER.warning(
                    "isaac sim is already running; changes to world config "
                    "(stage/headless/etc) require restarting the module"
                )
            return
        if self._boot_error is not None:
            raise RuntimeError(f"isaac sim failed to boot previously: {self._boot_error}")

        self.cfg = cfg
        self._boot_requested.set()
        if not self._booted.wait(timeout=cfg.boot_timeout):
            raise TimeoutError(f"isaac sim did not boot within {cfg.boot_timeout}s")
        if self._boot_error is not None:
            raise RuntimeError(f"isaac sim failed to boot: {self._boot_error}")

    def request_stop(self) -> None:
        self._stop.set()

    def main_loop(self) -> None:
        """Run forever on the owning (main) thread: wait for a boot request,
        boot, then step the sim while draining queued tasks."""
        self._sim_thread_id = threading.get_ident()

        while not self._stop.is_set() and not self._boot_requested.wait(timeout=0.1):
            self._drain_tasks()

        if self._stop.is_set():
            return

        try:
            self._boot()
        except BaseException as e:  # SimulationApp failures can be SystemExit
            LOGGER.exception("failed to boot isaac sim")
            self._boot_error = e
            self._booted.set()
            return
        self._booted.set()

        last = time.monotonic()
        loop = self._loop_s
        pace = self._pacer()
        prev = 0.0 if self.mock else float(self.world.current_time)
        while not self._stop.is_set():
            t0 = time.perf_counter()
            self._drain_tasks()
            t1 = time.perf_counter()
            now = time.monotonic()
            dt = now - last
            last = now
            if self.mock:
                for cb in list(self._step_callbacks.values()):
                    cb(dt)
                time.sleep(0.01)
                self._sim_clock += dt
            else:
                self.world.step(render=True)
                cur = float(self.world.current_time)
                self._sim_clock += cur - prev if cur >= prev else cur
                prev = cur
            loop["drain"] += t1 - t0
            loop["step"] += time.perf_counter() - t1
            loop["n"] += 1
            if pace is not None:
                pace()

        if self._sim_app is not None:
            try:
                self._sim_app.close()
            except Exception:
                LOGGER.exception("error closing isaac sim")

    def _pacer(self) -> Optional[Callable[[], None]]:
        """Hold the sim to `max_rtf` of the wall clock, draining submitted work while it
        waits. It never catches up: a sim that fell behind (a heavy render, a reset)
        re-anchors rather than running fast to make the time back."""
        cap = float(self.cfg.max_rtf) if self.cfg is not None else 0.0
        if self.mock or cap <= 0.0:
            return None
        anchor = {"wall": time.monotonic(), "sim": float(self.world.current_time)}

        def pace() -> None:
            sim = float(self.world.current_time)
            if sim < anchor["sim"]:
                anchor.update(wall=time.monotonic(), sim=sim)
                return
            while not self._stop.is_set():
                ahead = (sim - anchor["sim"]) / cap - (time.monotonic() - anchor["wall"])
                if ahead <= 0.0 or ahead > 1.0:
                    # Behind, or more than a second ahead: nothing paced steps that far
                    # ahead - `profile` did, and repaying it froze the sim for a minute
                    # with every camera's wake timing out (2026-10-14).
                    if ahead < -0.25 or ahead > 1.0:
                        anchor.update(wall=time.monotonic(), sim=sim)
                    return
                t = time.perf_counter()
                self._drain_tasks()
                self._loop_s["drain"] += time.perf_counter() - t
                time.sleep(min(ahead, 0.002))
        return pace

    def _drain_tasks(self) -> None:
        # Only what was queued when the drain began, then the loop steps. Drained until
        # empty, four camera streams and the cameras' own waits for a fresh frame - which
        # re-queue the moment they are answered - kept the queue from ever emptying: 9.94
        # of every 10.3 s draining, 3 steps, the sim at 0.01 of real time and every camera
        # waiting on the steps it was preventing (2026-10-14).
        for _ in range(max(1, self._tasks.qsize())):
            try:
                fn, fut = self._tasks.get_nowait()
            except queue.Empty:
                return
            if fut.set_running_or_notify_cancel():
                try:
                    fut.set_result(fn())
                except BaseException as e:
                    fut.set_exception(e)

    def run(self, fn: Callable[[], Any], timeout: float = 30.0) -> Any:
        """Run fn on the sim thread and return its result."""
        if threading.get_ident() == self._sim_thread_id:
            return fn()
        fut: Future = Future()
        self._tasks.put((fn, fut))
        return fut.result(timeout=timeout)

    # ------------------------------------------------------------------
    # boot
    # ------------------------------------------------------------------

    def _boot(self) -> None:
        cfg = self.cfg
        assert cfg is not None
        if cfg.mock:
            LOGGER.info("booting in MOCK mode - no isaac sim")
            self.mock = True
            return

        LOGGER.info("booting isaac sim (headless=%s)...", cfg.headless)
        try:
            from isaacsim import SimulationApp  # isaac sim >= 4.5
        except ImportError:
            from omni.isaac.kit import SimulationApp  # older releases

        # quiet kit's console stream; unknown argv entries are forwarded to kit
        level = cfg.kit_log_level.capitalize()
        sys.argv.append(f"--/log/outputStreamLevel={level}")

        streaming = cfg.livestream and cfg.headless
        launch: Dict[str, Any] = {"headless": cfg.headless}
        if streaming:
            # SimulationApp hides kit's UI whenever headless is set, which
            # leaves a connected streaming client looking at an empty frame.
            # These mirror isaacsim.exp.full.streaming and the livestream
            # standalone example: keep the UI, render it at a size worth
            # streaming, and show the default grid.
            launch.update(
                hide_ui=False,
                width=cfg.livestream_width,
                height=cfg.livestream_height,
                window_width=cfg.livestream_width,
                window_height=cfg.livestream_height,
                renderer="RaytracedLighting",
                display_options=3286,
            )

        self._sim_app = SimulationApp(launch)

        try:
            import carb.settings

            carb.settings.get_settings().set("/log/outputStreamLevel", level)
            # DLSS in Performance mode. 6.1 boots in it; 5.0 boots in Auto (3), which cost
            # 340 ms a step for one camera against 121 in Performance, with frames that look
            # the same (2026-10-17).
            carb.settings.get_settings().set("/rtx/post/dlss/execMode", 0)
        except Exception:
            pass

        if streaming:
            try:
                try:
                    from isaacsim.core.utils.extensions import enable_extension
                except ImportError:
                    from omni.isaac.core.utils.extensions import enable_extension

                ip = cfg.livestream_public_ip or _local_ip()
                self._sim_app.set_setting("/app/window/drawMouse", True)
                # the client asks to resize the stream as soon as it connects;
                # without this the request is refused and the view stays blank
                self._sim_app.set_setting("/app/livestream/allowResize", True)
                self._sim_app.set_setting("/app/livestream/port", 49100)
                if ip:
                    self._sim_app.set_setting("/app/livestream/publicEndpointAddress", ip)
                # 5.0 ships the streaming service under omni.services; 4.5 and
                # older only have the kit extension
                for ext in ("omni.services.livestream.nvcf", "omni.kit.livestream.webrtc"):
                    if enable_extension(ext):
                        break
                else:
                    raise RuntimeError("no livestream extension could be enabled")
                LOGGER.info(
                    "livestream enabled - connect the 'Isaac Sim WebRTC Streaming "
                    "Client' app to %s (TCP 49100 + UDP 47998 must be reachable)",
                    ip or "<this machine's IP>",
                )
            except Exception:
                LOGGER.exception("could not enable livestream; continuing without it")

        self._isaac = _import_isaac()

        if cfg.usd_stage:
            LOGGER.info("opening stage %s", cfg.usd_stage)
            self._isaac.open_stage(cfg.usd_stage)

        self.world = self._isaac.World(
            physics_dt=cfg.physics_dt,
            rendering_dt=cfg.rendering_dt,
            stage_units_in_meters=1.0,
        )
        self._add_ground(cfg)
        self._apply_viewport_grid(bool(cfg.render.get("viewport_grid", True)))
        self._apply_lighting(cfg.lighting)
        for prop in cfg.props:
            try:
                self._spawn_prop(prop)
            except Exception:
                LOGGER.exception("failed to spawn prop %s", prop.get("name"))
        self.world.reset()
        if not (cfg.livestream and cfg.headless):
            # Nothing shows the kit viewport without the livestream, but it still renders
            # every frame: 17 of an idle frame's 23 ms with the livestream off (2026-10-17,
            # L4). Off once, here - the cameras are their own render products.
            try:
                from omni.kit.viewport.utility import get_active_viewport
                vp = get_active_viewport()
                if vp is not None:
                    vp.updates_enabled = False
            except Exception:                                            # noqa: BLE001
                LOGGER.exception("could not turn the unwatched viewport off")
        LOGGER.info("isaac sim world ready")

    def _spawn_prop(self, prop: Dict[str, Any]) -> None:
        """Add a configured prop to the scene (runs on the sim thread,
        before the initial world.reset)."""
        import numpy as np

        from .spatial import to_vec3

        if not prop.get("name"):
            raise ValueError(f"every prop needs a name: {prop}")
        name = _prim_name(str(prop["name"]))
        prim_path = f"/World/{name}"
        position = list(to_vec3(prop.get("position")))
        kind = str(prop.get("type", "cube"))

        if kind == "usd":
            usd_path = prop.get("usd_path")
            if not usd_path:
                raise ValueError(f"prop {name}: type 'usd' needs usd_path")
            if self._usd_exists(usd_path) is False:
                raise ValueError(f"prop {name}: usd not found: {usd_path}")
            self._isaac.add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
            xform = self._isaac.SingleXFormPrim(prim_path)
            # Which way the asset faces. An asset is authored in its own frame and a cell
            # rarely wants it that way round: this mailbox is authored with its length
            # along local x and its flag on local +y, and the cell needs the length along
            # the belt, a flat clear face towards the arm, and the flag where the camera
            # can see it. Only yaw, because a part standing on a surface has one free
            # rotation and admitting the other two would invite poses that cannot rest.
            yaw = float(prop.get("yaw_deg", 0.0))
            # ...unless the ASSET says otherwise, in which case say what it says.
            #
            # The restriction above is right for a part this cell drops onto a tray: it
            # has one free rotation and the other two only invite poses that cannot rest.
            # It is wrong for a part whose pose is PUBLISHED. The mailbox's door lies flat
            # in the jig at `assembly:flatOrientation` (0.707107, 0, 0.707107, 0), which is
            # 90 degrees about Y, and no yaw lays a door down - so without this the cell
            # can only ever show the door shut, which is not the cell that was asked for.
            orient = prop.get("orientation_wxyz")
            # Authored ONTO THE PRIM, not through set_world_pose.
            #
            # A referenced asset arrives with its own xform ops, often
            # translate-orient-scale
            # with a non-identity orient. Writing a world pose over that leaves the asset's
            # own ops in place and composes with them, so the prop ends up somewhere that
            # depends on the asset rather than on the config. Clearing the order and
            # authoring translate-then-rotate makes the prop's pose the ONLY thing in its
            # transform, which is what the rest of this module assumes when it reads a
            # prop's bounds as offsets about its position.
            from pxr import Gf as _Gf, UsdGeom as _UsdGeom
            _stage = self._isaac.omni_usd.get_context().get_stage()
            _x = _UsdGeom.Xformable(_stage.GetPrimAtPath(prim_path))
            _x.ClearXformOpOrder()
            _x.AddTranslateOp().Set(_Gf.Vec3d(*[float(v) for v in position]))
            if orient is not None:
                w, qx, qy, qz = (float(v) for v in orient)
                # One op, not an orient composed with a rotate: a prop's transform is the
                # only thing in it by construction, and two rotation ops is how that stops
                # being true.
                # In the precision the asset already authored, if it did: a referenced
                # part that carries its own `xformOp:orient` as a quatd refuses a quatf op
                # of the same name ("has typeName 'quatd' which does not match").
                _have = _x.GetPrim().GetAttribute("xformOp:orient")
                if _have and str(_have.GetTypeName()) == "quatd":
                    _x.AddOrientOp(_UsdGeom.XformOp.PrecisionDouble).Set(
                        _Gf.Quatd(w, _Gf.Vec3d(qx, qy, qz)))
                else:
                    _x.AddOrientOp().Set(_Gf.Quatf(w, _Gf.Vec3f(qx, qy, qz)))
            elif yaw:
                _x.AddRotateZOp().Set(float(yaw))
            # Scale and mass were silently ignored on a usd prop: a cube honoured both and
            # a referenced asset honoured neither, so an asset authored at its real size
            # could only be used at its real size, and carried whatever mass its author
            # wrote. Both matter here - this cell is a teaching rig laid out around a part
            # a third of a real mailbox's length.
            scale = prop.get("scale")
            if scale is not None:
                values = ([float(scale)] * 3 if isinstance(scale, (int, float))
                          else [float(v) for v in scale])
                xform.set_local_scale(np.array(values))
            self._select_variants(prim_path, prop.get("variants") or {})
            # A part a feeder carries must never sleep. A sleeping body is not woken by a
            # surface velocity under it or by a teleport, so the flag sat motionless 0.7 mm
            # above its belt after every restage that let it settle.
            if prop.get("never_sleep"):
                from pxr import PhysxSchema as _Px, Usd as _Usd, UsdPhysics as _Up
                for _p in _Usd.PrimRange(_stage.GetPrimAtPath(prim_path)):
                    if _p.HasAPI(_Up.RigidBodyAPI):
                        _Px.PhysxRigidBodyAPI.Apply(_p).CreateSleepThresholdAttr(0.0)
            # A mechanism inside a fixed asset - the pin magazine's lifter - has to be a
            # KINEMATIC BODY to push anything. As a plain collider its moves are teleports
            # with no velocity: measured, the rod rose 20 mm under the pin and the pin
            # stayed put or was kicked 10 mm sideways. A kinematic body's xform change is
            # a kinematic target, which PhysX moves through with velocity.
            for _rel in prop.get("kinematic_parts") or []:
                from pxr import UsdPhysics as _Up
                _kp = _stage.GetPrimAtPath(f"{prim_path}/{_rel}")
                if not _kp or not _kp.IsValid():
                    raise ValueError(f"{prim_path}/{_rel}: no such part to make kinematic")
                _body = _Up.RigidBodyAPI.Apply(_kp)
                _body.CreateKinematicEnabledAttr().Set(True)
                LOGGER.info("prop %s: %s is a kinematic body", name, _rel)
            # Colliders, if the asset has none worth using. Measured on this mailbox: it
            # declares Collision scopes and a model variant that references colliders out
            # of its geometry file, and the composed stage gives those scopes NO EXTENT -
            # the references resolve to nothing. A rigid body with no collision shape does
            # not fall and cannot be gripped, so the suction raycast passed straight
            # through and the round carried the part nowhere for five presentations.
            #
            # `collision_from` names the scopes whose meshes become colliders - Visuals by
            # default, and deliberately NOT Decals, since a mark with a collider is a
            # permanent obstacle in the other arm's approach.
            # Optionally stop the asset being an articulation.
            #
            # A surface gripper grips by making a D6 joint to the rigid body its raycast
            # hits. The cube part is one plain rigid body and grips 3/3; this mailbox is an
            # articulation - an articulation root over a shell and a hinged door - and the
            # same grip on the same face with the same clearance reports Open every time.
            # Stripping the root leaves the two bodies and their hinge intact as an ordinary
            # jointed pair, solved in maximal coordinates rather than as an articulation,
            # which is what the gripper can attach to.
            if prop.get("articulation_root") is False:
                self._strip_articulation_root(prim_path)
            # Scenery must not be able to move. A cube prop gets no rigid body unless it
            # asks for one, but a referenced asset brings whatever its author gave it,
            # and `fixed` was only ever read when reporting obstacles - so the conveyor
            # spawned as a dynamic body, fell through its own legs and came to rest
            # tilted 18 degrees with its deck sloping from 207 mm to 759 mm along its
            # length. A prop declared fixed is now made fixed in the simulator too.
            # Parts of a referenced asset to switch off: the warehouse's own ground plane (a
            # second floor collider at z 0) and its distant lights (the insets' exposure was
            # set against this cell's dome).
            if prop.get("lights_off"):
                # Every light the asset brings: the warehouse's 37 ceiling lamps washed the
                # deck out white under this cell's own dome (2026-10-13).
                from pxr import Usd as _U2, UsdLux as _Lux
                for _lp in _U2.PrimRange(_stage.GetPrimAtPath(prim_path)):
                    if _lp.HasAPI(_Lux.LightAPI) or _lp.GetTypeName().endswith("Light"):
                        _lp.SetActive(False)
            # Whole families of children by name - the warehouse's 72 roof panels, which
            # shut out the dome light the cell is lit by (2026-10-13).
            _pref = tuple(str(v) for v in (prop.get("deactivate_prefix") or []))
            if _pref:
                _root = _stage.GetPrimAtPath(prim_path)
                _kids = list(_root.GetChildren())
                if len(_kids) == 1 and not _kids[0].GetName().startswith(_pref):
                    _kids += list(_kids[0].GetChildren())
                for _k in _kids:
                    if _k.GetName().startswith(_pref):
                        _k.SetActive(False)
            for rel in prop.get("deactivate") or []:
                sub = _stage.GetPrimAtPath(f"{prim_path}/{rel}")
                if sub and sub.IsValid():
                    sub.SetActive(False)
                else:
                    LOGGER.warning("prop %s: no %s to deactivate", name, rel)
            if prop.get("fixed"):
                self._make_static(prim_path)
            approximation = prop.get("collision_approximation")
            if approximation:
                self._add_usd_colliders(
                    prim_path, str(approximation),
                    [str(v) for v in (prop.get("collision_from") or ["Visuals"])])
            # Mass last: it is spread over the rigid bodies by their size, and their size
            # is only known once the geometry a variant brings in is composed.
            mass = prop.get("mass_kg")
            if mass is not None:
                self._set_usd_mass(prim_path, float(mass))
            if prop.get("conveyor"):
                self._setup_conveyor(str(prop["name"]), prim_path, prop)
            return

        if prop.get("children"):
            self._spawn_composite_prop(prop, name, prim_path, position)
            return

        if kind != "cube":
            raise ValueError(
                f"prop {name}: unknown type {kind!r} (cube, usd, or a cube with children)"
            )

        kwargs: Dict[str, Any] = dict(
            prim_path=prim_path,
            name=name,
            position=np.array(position),
            size=float(prop.get("size", 0.05)),
        )
        if prop.get("scale") is not None:
            kwargs["scale"] = np.array([float(v) for v in prop["scale"]])
        if prop.get("color") is not None:
            kwargs["color"] = np.array([float(v) for v in prop["color"]])
        # A cube may be yawed. Needed for anything built from boxes that does not run along
        # the cell's axes - the flag's feeder comes in at an angle to the main belt.
        if prop.get("yaw_deg"):
            kwargs["orientation"] = _yaw_quat(float(prop["yaw_deg"]))
        cls = self._isaac.FixedCuboid if prop.get("fixed") else self._isaac.DynamicCuboid
        self.world.scene.add(cls(**kwargs))
        if prop.get("friction") is not None:
            self._set_friction(prim_path, prop["friction"])
        # A cube can be a BELT. A conveyor's deck is a flat rectangle and a cube prop is a
        # flat rectangle; the only reason this hook lived solely on the referenced-asset
        # path is that every conveyor so far happened to arrive as a usd asset with its
        # deck in a named child. Driving a 4.6 m deck should not require modelling it as
        # an asset first.
        if prop.get("conveyor"):
            self._setup_conveyor(str(prop["name"]), prim_path, prop)

    def _set_friction(self, prim_path: str, friction: Any) -> Dict[str, float]:
        """Give a prop its own surface, because default friction is not a neutral choice.

        A conveyor moves a part by friction and its side guides resist by friction, and
        with one default material on both the guides win: measured, a reset-then-feed
        delivered the part 2 times in 6, the rest binding against the rails and stopping
        with the rollers still turning under them. That is not a physics quirk to tune
        around - it is what a real line solves by making the guides UHMW and the belt
        rubber. The numbers belong in the config where the rest of the cell is described.
        """
        from pxr import UsdPhysics, UsdShade

        values = ({"static": float(friction), "dynamic": float(friction)}
                  if isinstance(friction, (int, float)) else
                  {str(k): float(v) for k, v in dict(friction).items()})
        stage = self._isaac.omni_usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            raise ValueError(f"{prim_path}: not in the stage, cannot set friction")
        material = UsdShade.Material.Define(stage, f"{prim_path}/PhysicsMaterial")
        physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics.CreateStaticFrictionAttr().Set(values.get("static", 0.5))
        physics.CreateDynamicFrictionAttr().Set(
            values.get("dynamic", values.get("static", 0.5)))
        physics.CreateRestitutionAttr().Set(values.get("restitution", 0.0))
        UsdShade.MaterialBindingAPI.Apply(prim)
        UsdShade.MaterialBindingAPI(prim).Bind(
            material, UsdShade.Tokens.weakerThanDescendants, "physics")
        LOGGER.info("prop %s: friction %s", prim_path, values)
        return values

    _CHILD_SHAPES = ("cube", "cylinder", "capsule", "cone", "sphere")
    _jaw_specs: Dict[str, Dict[str, Any]] = {}
    # Each referenced finger prim's own rest translate, captured the first time it moves,
    # so the opening is measured from where the ASSET put the finger rather than from the
    # last place this code left it - which would otherwise drift a little every cycle.
    _tool_rest: Dict[str, List[float]] = {}

    def _select_variants(self, prim_path: str, wanted: Dict[str, Any]) -> None:
        """Choose this prop's usd variants.

        An asset that ships its own variation - paint scheme, which defect is on it,
        whether the door binds - is describing states the cell wants to drive, and
        driving them through variants means the cell never has to know how the asset is
        built. Named here rather than discovered, so a typo is an error rather than a
        prop that silently stays as it was.
        """
        if not wanted:
            return
        stage = self._isaac.omni_usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            raise ValueError(f"{prim_path}: not in the stage, cannot select variants")
        sets = prim.GetVariantSets()
        known = list(sets.GetNames())
        for name, value in wanted.items():
            if name not in known:
                raise ValueError(
                    f"{prim_path}: no variant set {name!r}; it has {known}")
            variant_set = sets.GetVariantSet(name)
            options = list(variant_set.GetVariantNames())
            if str(value) not in options:
                raise ValueError(
                    f"{prim_path}: variant set {name!r} has no {value!r}; "
                    f"options are {options}")
            variant_set.SetVariantSelection(str(value))
        LOGGER.info("prop %s: variants %s", prim_path, dict(wanted))

    def variants(self, prop_name: str) -> Dict[str, str]:
        """Which variant each of a prop's variant sets is currently on."""
        self._require_booted()
        stage = self._isaac.omni_usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(f"/World/{_prim_name(prop_name)}")
        if not prim or not prim.IsValid():
            raise ValueError(f"no prop named {prop_name!r}")
        sets = prim.GetVariantSets()
        return {name: sets.GetVariantSet(name).GetVariantSelection()
                for name in sets.GetNames()}

    def set_variants(self, prop_name: str, wanted: Dict[str, Any],
                     render_steps: Optional[int] = None) -> Dict[str, str]:
        """Switch a prop's variants, and wait for the change to be drawn.

        Same reason `set_defects` waits: a switch followed straight away by a frame grab
        hands back the render from before the switch, which is a mislabelled image and
        the kind nobody notices.
        """
        self._require_booted()
        path = f"/World/{_prim_name(prop_name)}"

        def _apply():
            self._select_variants(path, wanted)
            for _ in range(int(render_steps or self.DEFECT_RENDER_STEPS)):
                self.world.step(render=True)

        self.run(_apply)
        return self.variants(prop_name)

    def set_light(self, path: str, intensity: float,
                  render_steps: Optional[int] = None) -> Dict[str, Any]:
        """Set a light's intensity (a lamp, or a tool's flash) and wait for it to be drawn:
        how an exposure is tuned in one pass rather than one restart per value."""
        self._require_booted()
        if self.mock:
            return {"path": path, "intensity": intensity}
        from pxr import UsdLux

        def _apply():
            stage = self._isaac.omni_usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                raise ValueError(f"{path}: not in the stage")
            UsdLux.LightAPI(prim).GetIntensityAttr().Set(float(intensity))
            for _ in range(int(render_steps or self.DEFECT_RENDER_STEPS)):
                self.world.step(render=True)

        self.run(_apply)
        return {"path": path, "intensity": float(intensity)}

    # An asset may carry its paint defects as a variant set rather than as children. The
    # cube part spawned a cube per mark and switched them by visibility; this mailbox has
    # one `paint_defect` set whose options are the marks, so the part wears exactly one at
    # a time. The verbs below read and write whichever of the two a prop actually is, so
    # the cell asks the same question of either.
    PAINT_VARIANT_SET = "paint_defect"
    NO_PAINT_DEFECT = "none"

    def _paint_variants(self, prim):
        """(variantSet, [option names]) for a prop's paint defects, or None."""
        sets = prim.GetVariantSets()
        if not sets.HasVariantSet(self.PAINT_VARIANT_SET):
            return None
        chosen = sets.GetVariantSet(self.PAINT_VARIANT_SET)
        return chosen, [str(n) for n in chosen.GetVariantNames()]

    def prop_attributes(self, prop_name: str, names: List[str]) -> Dict[str, Any]:
        """Read usd attributes off a prop - an asset's own ground truth.

        The mailbox carries what it is on itself: which scheme, which paint defect,
        whether it is meant to pass. Reading that back is better than the cell
        remembering what it asked for, because the asset is what the camera sees.
        """
        self._require_booted()
        stage = self._isaac.omni_usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(f"/World/{_prim_name(prop_name)}")
        if not prim or not prim.IsValid():
            raise ValueError(f"no prop named {prop_name!r}")
        out: Dict[str, Any] = {}
        for attribute in names:
            handle = prim.GetAttribute(attribute)
            if not handle or not handle.IsValid():
                out[attribute] = None
                continue
            value = handle.Get()
            out[attribute] = (list(value) if hasattr(value, "__len__")
                              and not isinstance(value, str) else value)
        return out

    def _spawn_composite_prop(
        self, prop: Dict[str, Any], name: str, prim_path: str, position: List[float]
    ) -> None:
        """A prop made of several shapes that behave as one part.

        A real part is rarely a box. A stamped shell has a formed crown, a door, a
        flag - and a defect mark is a patch on one of its faces. Both are the same
        problem: several pieces of geometry that have to move as one thing.

        So the parent carries the rigid body and the children carry only geometry and
        collision. A child with its own RigidBodyAPI would be a separate part that
        happens to start nearby, and it would fall off the moment the sim stepped.

        The parent's own `size`/`scale` are not drawn. They stay the part's bounding
        box, which is what `prop_obstacles` reports to the planner, so a composite is
        still one obstacle rather than a cloud of them.
        """
        from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

        from .spatial import to_vec3

        stage = self._isaac.omni_usd.get_context().get_stage()
        root = UsdGeom.Xform.Define(stage, prim_path)
        root_prim = root.GetPrim()
        root.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in position]))

        fixed = bool(prop.get("fixed"))
        if not fixed:
            UsdPhysics.RigidBodyAPI.Apply(root_prim)
            mass = UsdPhysics.MassAPI.Apply(root_prim)
            mass.CreateMassAttr(float(prop.get("mass_kg", 0.4)))
            PhysxSchema.PhysxRigidBodyAPI.Apply(root_prim)

        default_color = prop.get("color")
        seen: Dict[str, str] = {}
        for index, child in enumerate(prop["children"]):
            shape = str(child.get("shape", "cube"))
            if shape not in self._CHILD_SHAPES:
                raise ValueError(
                    f"prop {name}: child {index} has unknown shape {shape!r}; "
                    f"one of {self._CHILD_SHAPES}"
                )
            child_name = _prim_name(str(child.get("name", f"part_{index}")))
            # A sanitised name that collides with a sibling is a child that SILENTLY
            # REPLACES another, and the result looks like a physics problem rather than a
            # naming one. `_prim_name` maps every character usd will not take to "_", so
            # "lip_-y" and "lip_+y" both become "lip__y": a fixture authored with a wall on
            # each side got one wall and nothing opposite it, and the part walked that way,
            # about 0.2 mm a cycle, for as long as anyone measured it. The same pair of
            # names had already turned a two-rail saddle into a one-rail saddle, which read
            # as "two narrow rails cannot hold a tall part" and was nothing of the kind.
            if child_name in seen:
                raise ValueError(
                    f"prop {name}: children {seen[child_name]!r} and "
                    f"{child.get('name')!r} both sanitise to the prim name "
                    f"{child_name!r}. Usd prim names take only letters, digits and "
                    f"underscores, so '+' and '-' are indistinguishable once sanitised - "
                    f"name them apart.")
            seen[child_name] = str(child.get("name", f"part_{index}"))
            child_path = f"{prim_path}/{child_name}"
            if shape == "cube":
                geom = UsdGeom.Cube.Define(stage, child_path)
                geom.CreateSizeAttr(1.0)
                extent = 0.5
            elif shape == "sphere":
                geom = UsdGeom.Sphere.Define(stage, child_path)
                geom.CreateRadiusAttr(float(child.get("radius", 0.05)))
                extent = float(child.get("radius", 0.05))
            else:
                define = {"cylinder": UsdGeom.Cylinder, "capsule": UsdGeom.Capsule,
                          "cone": UsdGeom.Cone}[shape]
                geom = define.Define(stage, child_path)
                geom.CreateRadiusAttr(float(child.get("radius", 0.05)))
                geom.CreateHeightAttr(float(child.get("height", 0.1)))
                geom.CreateAxisAttr(str(child.get("axis", "Z")))
                extent = max(float(child.get("radius", 0.05)),
                             float(child.get("height", 0.1)) / 2.0)
            # An implicit prim keeps its default extent unless told otherwise, which
            # leaves bounds - and anything computed from them - describing a unit shape.
            geom.CreateExtentAttr([Gf.Vec3f(-extent, -extent, -extent),
                                   Gf.Vec3f(extent, extent, extent)])

            xform = UsdGeom.Xformable(geom.GetPrim())
            xform.AddTranslateOp().Set(Gf.Vec3d(*to_vec3(child.get("position"))))
            rotation = child.get("rotation_xyz_deg")
            if rotation:
                xform.AddRotateXYZOp().Set(Gf.Vec3f(*[float(v) for v in rotation]))
            if shape == "cube":
                scale = child.get("scale") or (child.get("size", 0.05),) * 3
                xform.AddScaleOp().Set(Gf.Vec3f(*[float(v) for v in scale]))

            # Structural children collide; marks on a surface do not. A defect is
            # paint, and USD visibility is a render attribute only - PhysX keeps the
            # collider whichever way the flag is set, so a hidden "proud door" would
            # still protrude into the other arm's approach and the suction raycast
            # would still hit it.
            if child.get("collision", True):
                UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
            colour = child.get("color", default_color)
            if colour is not None:
                geom.CreateDisplayColorAttr(
                    [Gf.Vec3f(*[float(v) for v in colour])])

        LOGGER.info("prop %s: composite of %d shapes, %s",
                    name, len(prop["children"]), "fixed" if fixed else "dynamic")

    def _require_booted(self) -> None:
        if not self._booted.is_set():
            raise RuntimeError(
                "isaac sim world is not running - configure a "
                "hackathons:isaac-sim:world component and depend on it"
            )
        if self._boot_error is not None:
            raise RuntimeError(f"isaac sim failed to boot: {self._boot_error}")

    # ------------------------------------------------------------------
    # world controls (used by the world component's DoCommand)
    # ------------------------------------------------------------------

    def play(self) -> None:
        self._require_booted()
        if not self.mock:
            self.run(lambda: self.world.play())

    def pause(self) -> None:
        self._require_booted()
        if not self.mock:
            self.run(lambda: self.world.pause())

    def reset(self) -> None:
        """Put the cell back to the top of a round.

        `world.reset()` restores what is registered in `world.scene`, which is the arms
        and the cube props and NOT a referenced asset - so the mailbox stayed wherever
        the last round left it, and a conveyor kept running with its rollers still
        turning. Every consumer here resets and then expects a pickable part on the
        infeed; without this they would get one 1.2 m upstream, or wherever it jammed.

        This passed unnoticed for as long as it did because the only test of `reset` was
        one where the part had never moved.
        """
        self._require_booted()
        self._index_state = None
        if self.mock:
            return
        for name in list(self._conveyors):
            self.conveyor(name, run=False)
        self.run(lambda: self.world.reset())
        self._send_arms_home()
        self._restage_props()

    def _restage_props(self) -> None:
        """Return every movable referenced asset to its configured pose.

        Dropped the last few millimetres rather than placed flush: put down with its
        underside exactly on the surface, PhysX reads the contact as a penetration and
        shoves the part out of it, which sent the mailbox upstream twice before a run
        had started.
        """
        for prop in (self.cfg.props or []):
            if str(prop.get("type", "cube")) != "usd" or prop.get("fixed"):
                continue
            name = str(prop.get("name", ""))
            where = [float(v) * 1000.0 for v in (prop.get("position") or (0, 0, 0))]
            try:
                self.place_prop(name, position_mm=[where[0], where[1], where[2] + 5.0])
            except Exception:  # noqa: BLE001 - a reset that half worked must say so
                LOGGER.exception("reset: could not re-stage %s", name)
                raise

    def _send_arms_home(self) -> None:
        """Put every arm that declares a home back in it.

        `world.reset()` returns each articulation to the joint state the usd was
        authored with, which for a ur5e is all zeros - and all zeros points the arm
        straight out across the cell. In a cell with anything in that swath the arm
        spawns intersecting it, cannot move, and reports a stuck joint on the very
        first command of the round: measured, arm-b could not even reach its home pose
        because its rest pose went through an outfeed conveyor's support.

        A cell describes where its arms rest. Nothing else here should have to arrange
        the scenery around a pose the asset happened to ship with.
        """
        for name, (attrs, handle) in list(self._handles.items()):
            home = attrs.get("home_joints_deg")
            if not home or not isinstance(handle, IsaacArmHandle):
                continue
            try:
                radians = _as_radians(home)
                self.run(lambda h=handle, r=radians: h._art.set_joint_positions(r))
                handle.set_joint_targets([float(v) for v in radians])
                LOGGER.info("arm %s reset to its home configuration", name)
            except Exception:
                LOGGER.exception("could not send arm %s home", name)

    def _add_ground(self, cfg: "SimConfig") -> None:
        """Author whatever floor `ground_plan` decided on, before props spawn."""
        kind, kwargs = ground_plan(cfg.ground, cfg.usd_stage)
        if kind == "skip":
            LOGGER.warning("ground config ignored: %s", kwargs["reason"])
            return
        if kind == "none":
            return
        if kind == "plane":
            import numpy as np

            # PreviewSurface calls color.tolist(), so the colour has to be an array
            kwargs["color"] = np.array(kwargs["color"], dtype=float)
            try:
                self.world.scene.add_ground_plane(
                    name=GROUND_PLANE_NAME,
                    prim_path=GROUND_PLANE_PRIM_PATH,
                    z_position=0.0,
                    **kwargs,
                )
                if ground_is_matte(cfg.ground):
                    self._make_ground_matte()
            except Exception:
                LOGGER.exception("could not add the ground plane; falling back to the grid")
                self.world.scene.add_default_ground_plane()
            return
        if not cfg.usd_stage:
            self.world.scene.add_default_ground_plane()

    def _make_ground_matte(self) -> None:
        """Flag the plane's prims as RTX matte objects: invisible to primary rays, still
        catching shadows. Best-effort, so a render-settings change cannot block boot."""
        try:
            import carb
            from pxr import Sdf, Usd, UsdGeom

            stage = self._isaac.omni_usd.get_context().get_stage()
            root = stage.GetPrimAtPath(GROUND_PLANE_PRIM_PATH)
            for prim in (list(Usd.PrimRange(root)) if root.IsValid() else []):
                if prim == root or prim.IsA(UsdGeom.Mesh):
                    UsdGeom.PrimvarsAPI(prim).CreatePrimvar(
                        "isMatteObject", Sdf.ValueTypeNames.Bool).Set(True)
            settings = carb.settings.get_settings()
            settings.set(MATTE_OBJECT_SETTING, True)
            settings.set(SHADOW_CATCHER_SETTING, True)
        except Exception:
            LOGGER.exception("failed to make the ground plane matte")

    def _apply_viewport_grid(self, show_grid: bool) -> None:
        """Toggle the viewport's grid overlay. Best-effort; never raises."""
        try:
            import carb

            carb.settings.get_settings().set(VIEWPORT_GRID_SETTING, show_grid)
            LOGGER.info("set %s to %s", VIEWPORT_GRID_SETTING, show_grid)
        except Exception:
            LOGGER.exception("failed to apply render.viewport_grid")

    def _apply_lighting(self, lighting: Dict[str, Any]) -> None:
        """Dome and key lights. Best-effort, so bad lighting cannot block boot.

        A dome is what stops a scene reading as objects on a void: it lights every surface
        from every direction the way a room does, and an untextured one still beats the
        single default light.
        """
        if not lighting:
            return
        try:
            from pxr import Gf, Sdf, UsdGeom, UsdLux

            stage = self._isaac.omni_usd.get_context().get_stage()
            dome = lighting.get("dome")
            if dome is not None:
                light = UsdLux.DomeLight.Define(stage, DOME_LIGHT_PRIM_PATH)
                light.CreateIntensityAttr(
                    float(dome.get("intensity", DEFAULT_DOME_INTENSITY)))
                light.CreateColorAttr(Gf.Vec3f(
                    *[float(v) for v in dome.get("color", DEFAULT_DOME_COLOR)]))
                texture = dome.get("texture")
                if texture:
                    light.CreateTextureFileAttr(Sdf.AssetPath(str(texture)))
                    light.CreateTextureFormatAttr(
                        str(dome.get("texture_format", "latlong")))
                rotation = dome.get("rotation_deg")
                if rotation is not None:
                    xformable = UsdGeom.Xformable(light)
                    # clear first, so a re-apply replaces rather than stacks
                    xformable.ClearXformOpOrder()
                    xformable.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, float(rotation)))
            sphere = lighting.get("sphere_intensity")
            if sphere is not None:
                prim = stage.GetPrimAtPath(SPHERE_LIGHT_PRIM_PATH)
                if prim.IsValid():
                    UsdLux.SphereLight(prim).GetIntensityAttr().Set(float(sphere))
            self._pending_lamps = list(lighting.get("lamps") or [])
            self._apply_lamps(stage, self._pending_lamps)
        except Exception:
            LOGGER.exception("failed to apply scene lighting")

    def _apply_lamps(self, stage, lamps: List[Dict[str, Any]]) -> None:
        """Named lights, optionally parented to a moving prim - an inspection flash.

        A dome lights a room; it does not light the inside of a mailbox. The interior shot
        puts a fisheye 70 mm through a 160 x 216 mm aperture and comes back black, which is
        the correct image of an unlit cavity and not a camera fault. A real interior
        inspection carries its own light, so a lamp here can hang off the same flange its
        camera does and travel in with it.

        Parented the same way a camera is, so one rule covers both: give `parent_prim` and
        the lamp is defined underneath it and `local_position` is in THAT prim's frame.
        """
        from pxr import Gf, Sdf, UsdGeom, UsdLux

        still_pending = []
        for spec in lamps:
            name = str(spec.get("name") or "lamp")
            parent = spec.get("parent_prim")
            if parent:
                # The arms do not exist yet at world boot, so a flash on a flange CANNOT
                # be created when the rest of the lighting is. The first version lost all
                # of them to that: `_require_prim` raised, the surrounding try/except
                # logged "failed to apply scene lighting", and the interior shot stayed
                # black with nothing in the config to suggest why. So a lamp whose parent
                # has not appeared yet is kept, not dropped, and `create_arm` retries.
                if not stage.GetPrimAtPath(str(parent)).IsValid():
                    still_pending.append(spec)
                    continue
                path = f"{str(parent).rstrip('/')}/{_prim_name(name)}"
            else:
                path = f"/World/{_prim_name(name)}"
            kind = str(spec.get("kind", "sphere")).lower()
            light = (UsdLux.DiskLight.Define(stage, path) if kind == "disk"
                     else UsdLux.SphereLight.Define(stage, path))
            light.CreateIntensityAttr(float(spec.get("intensity", 40000.0)))
            light.CreateRadiusAttr(float(spec.get("radius_m", 0.02)))
            light.CreateColorAttr(Gf.Vec3f(
                *[float(v) for v in spec.get("color", (1.0, 1.0, 1.0))]))
            # A lamp inside a cavity must not also be a visible white ball in the shot.
            prim = light.GetPrim()
            prim.CreateAttribute("visibleInPrimaryRay",
                                 Sdf.ValueTypeNames.Bool).Set(False)
            xform = UsdGeom.Xformable(light)
            xform.ClearXformOpOrder()          # clear first: re-apply replaces, not stacks
            pos = [float(v) for v in spec.get("local_position", (0.0, 0.0, 0.0))]
            xform.AddTranslateOp().Set(Gf.Vec3d(*pos))
            rpy = spec.get("local_orientation_rpy_deg")
            if rpy is not None:
                xform.AddRotateXYZOp().Set(Gf.Vec3f(*[float(v) for v in rpy]))
            LOGGER.info("lamp %s at %s intensity %s", name, path,
                        spec.get("intensity", 40000.0))
        self._pending_lamps = still_pending

    def prop_obstacles(self) -> List[Dict[str, Any]]:
        """Every prop as a box the viam motion service can treat as an obstacle.

        Props are spawned into isaac and exist nowhere in viam's frame system, so the
        planner cannot see them: asked to put a flange inside the belt it plans straight
        there, and the arm drives in until the collider stalls it. Callers have to pass
        these in `world_state` on every Move.

        Reported rather than published as component geometry because `fixed` is not a
        static property of the scene. A fixed belt is an obstacle forever; a dynamic part
        is an obstacle while it sits on the belt and becomes part of the *moving arm* once
        it is grasped - the same prop, two roles, switching mid-round. Only the caller
        knows which, so it gets the facts and decides.

        A dynamic prop is reported where it IS, not where it was spawned. This used to
        read the config, so once the part left the belt every obstacle list said it was
        still on the belt: the planner reasoned about a phantom there and knew nothing
        about the real part on the handoff stand or in a tray. Nothing collided, because
        the arms happen not to go back to the belt, but every plan after the pick was
        made against a scene that was wrong.

        Lengths are millimetres, because that is what viam's geometry messages use. A
        cube prop's edge is `size` scaled per axis, so dims are size * scale.
        """
        cfg = self.cfg
        if cfg is None:
            return []
        out: List[Dict[str, Any]] = []

        # The floor counts. boot() calls add_default_ground_plane() for any scene without
        # its own usd_stage, so there is a collider at z=0 that the planner knows nothing
        # about - and an arm on a pedestal has plenty of configurations that reach below
        # its own base. Left unreported, the planner routes an elbow into the ground and
        # the arm stalls pressing against it, which reads as a mysterious failure to
        # settle rather than as a collision.
        if not cfg.usd_stage:
            out.append({
                "label": "ground",
                "fixed": True,
                "center_mm": [0.0, 0.0, -50.0],
                "dims_mm": [20000.0, 20000.0, 100.0],
            })

        movable = [str(p.get("name", "")) for p in cfg.props
                   if not p.get("fixed", False)]
        live: Dict[str, Dict[str, Any]] = {}
        if movable:
            try:
                live = self.prop_poses(movable)
            except Exception:  # noqa: BLE001 - spawn positions beat no obstacles at all
                LOGGER.exception("could not read live prop poses; using spawn positions")

        for prop in cfg.props:
            name = str(prop.get("name", ""))
            if prop.get("obstacle") is False:
                # Scenery that surrounds the cell (the warehouse): measured, it is one box
                # the size of a building with every arm inside it.
                continue
            if str(prop.get("type", "cube")) != "cube":
                # A usd prop's extent is whatever the asset says, and the module will
                # not guess. But an asset that states its own collision bounds - this
                # cell's mailbox does, per paint scheme - can be reported like anything
                # else, and must be: an obstacle the planner cannot see is one the arm
                # drives into.
                bounds = prop.get("bounds_m") or self._measured_bounds(name)
                if not bounds:
                    # Nothing measurable and nothing declared. Say so rather than drop it:
                    # skipping silently is how the part disappeared from the obstacle list
                    # the moment its declared bounds were removed, and an obstacle the
                    # planner cannot see is one the arm drives into.
                    raise ValueError(
                        f"prop {name!r} is a usd asset with no bounds_m and nothing "
                        f"measurable in the stage, so the planner would never see it")
                here = live.get(name, {}).get("position_mm")
                origin = ([float(v) for v in here] if here is not None
                          else [float(v) * 1000.0
                                for v in (prop.get("position") or (0, 0, 0))])
                # One asset is not always one box. A conveyor's single measured extent
                # runs from the floor to the tops of its posts and from rail to rail, so
                # the deck the part rests on and the pose the cup grips at are both
                # *inside* it: the planner refuses the pick as a goal in collision, and
                # the same box refuses the place at the other end. An asset may therefore
                # declare a LIST of boxes - deck, each rail, each post - and each is
                # reported separately, suffixed so the whole prop can still be named.
                # The boxes are in the ASSET's frame, so a yawed prop's boxes turn with
                # it: each centre is rotated about the origin and the box reported yawed.
                # Placed unturned, the reject spur (yaw -90) put its side guide across
                # the spot arm-c sets a shell down, 450 mm from where it really stands.
                yaw = math.radians(float(prop.get("yaw_deg") or 0.0))
                cy, sy = math.cos(yaw), math.sin(yaw)
                for box in (bounds if isinstance(bounds, list) else [bounds]):
                    low = [float(v) for v in box["min"]]
                    high = [float(v) for v in box["max"]]
                    part_of = str(box.get("name", "")) if isinstance(box, dict) else ""
                    mid = [(low[i] + high[i]) / 2.0 * 1000.0 for i in range(3)]
                    out.append({
                        "label": f"{name}.{part_of}" if part_of else name,
                        "fixed": bool(prop.get("fixed", False)),
                        "center_mm": [origin[0] + cy * mid[0] - sy * mid[1],
                                      origin[1] + sy * mid[0] + cy * mid[1],
                                      origin[2] + mid[2]],
                        "dims_mm": [(high[i] - low[i]) * 1000.0 for i in range(3)],
                        "yaw_deg": float(prop.get("yaw_deg") or 0.0),
                    })
                continue
            size = float(prop.get("size", 0.05))
            scale = prop.get("scale") or (1.0, 1.0, 1.0)
            scale = [float(v) for v in scale]
            here = live.get(name, {}).get("position_mm")
            if here is None:
                position = [float(v) for v in (prop.get("position") or (0.0, 0.0, 0.0))]
                here = [v * 1000.0 for v in position]
            out.append({
                "label": name,
                "fixed": bool(prop.get("fixed", False)),
                "center_mm": [float(v) for v in here],
                "dims_mm": [size * scale[i] * 1000.0 for i in range(3)],
                # A yawed box is not the axis-aligned box its dims describe; the caller
                # has to turn it, and only knows to if it is told.
                "yaw_deg": float(prop.get("yaw_deg") or 0.0),
            })
        return out

    def prop_poses(self, names: Optional[List[str]] = None) -> Dict[str, Dict[str, Any]]:
        """Where every prop actually is right now, as the sim sees it.

        Config says where a prop was *spawned*; this says where it *is*. Those differ the
        moment anything moves - a dynamic prop settles under gravity, slides, or gets
        carried by an arm - so anything reasoning about the current scene has to ask
        rather than read the config back.

        This is what lets a vision service be checked against ground truth instead of
        against itself: the part's true pose comes from the simulator, and a detector's
        answer can be scored against it without a human labelling frames.

        Positions are millimetres, to match viam's geometry and pose messages.
        Orientation is the prim's (w,x,y,z) quaternion.
        """
        self._require_booted()
        wanted = set(names) if names else None
        cfg = self.cfg
        configured = {str(p.get("name")): p for p in (cfg.props if cfg else [])}
        chosen = [n for n in configured if wanted is None or n in wanted]

        if self.mock:
            # No stage to read, so report where the prop was put. Honest for a scene
            # where nothing moves, and it keeps callers working without isaac.
            return {
                name: {
                    "position_mm": [float(v) * 1000.0
                                    for v in (configured[name].get("position")
                                              or (0.0, 0.0, 0.0))],
                    "orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
                    "live": False,
                }
                for name in chosen
            }

        def _read() -> Dict[str, Dict[str, Any]]:
            out: Dict[str, Dict[str, Any]] = {}
            for name in chosen:
                prim_path = f"/World/{_prim_name(name)}"
                # An asset can put its rigid body on a child rather than on the prim the
                # cell references - the mailbox is an articulation whose Body and Door
                # are separate bodies under one root - so a prop may name which child
                # carries the pose that matters.
                body = configured.get(name, {}).get("body_prim")
                if body:
                    prim_path = f"{prim_path}/{str(body).lstrip('/')}"
                # A simulated body's live pose is in the physics view, not necessarily in
                # USD: reading the xform can hand back the last value written to the
                # stage while the body has long since moved. That is not a small error -
                # it made a part look like it never left the belt while an arm carried it
                # away. Ask the rigid-body view first and only fall back to the xform.
                reader = None
                # Only for a prim that IS a rigid body. Wrapping any other prim in a
                # SingleRigidPrim applies a dynamic RigidBodyAPI to it - asked for every
                # prop, that turned the ground, the belt and the nest into falling bodies.
                from pxr import UsdPhysics as _UsdPhysics
                _prim = self._isaac.omni_usd.get_context().get_stage().GetPrimAtPath(prim_path)
                if (self._isaac.SingleRigidPrim is not None and _prim and _prim.IsValid()
                        and _prim.HasAPI(_UsdPhysics.RigidBodyAPI)):
                    try:
                        reader = self._isaac.SingleRigidPrim(prim_path)
                    except Exception:
                        reader = None
                if reader is None:
                    reader = self._isaac.SingleXFormPrim(prim_path)
                try:
                    position, quat = reader.get_world_pose()
                except Exception:
                    LOGGER.exception("could not read pose of prop %s", name)
                    continue
                out[name] = {
                    "position_mm": [float(v) * 1000.0 for v in position],
                    "orientation_wxyz": [float(v) for v in quat],
                    "live": True,
                }
            return out

        return self.run(_read)

    DEFECT_PREFIX = "defect_"

    def defects(self, prop_name: str) -> Dict[str, bool]:
        """Which of a prop's defect children exist right now.

        A defect is a child named `defect_*`. Reading the flag back is only a record of
        what was asked for, never evidence the camera could see anything - that is the
        annotator's job - but the round needs to know what it set.
        """
        self._require_booted()
        stage = self._isaac.omni_usd.get_context().get_stage()
        parent = stage.GetPrimAtPath(f"/World/{_prim_name(prop_name)}")
        if not parent or not parent.IsValid():
            raise ValueError(f"no prop named {prop_name!r}")
        painted = self._paint_variants(parent)
        if painted is not None:
            chosen, options = painted
            showing = str(chosen.GetVariantSelection())
            return {f"{self.DEFECT_PREFIX}{name}": name == showing
                    for name in options if name != self.NO_PAINT_DEFECT}
        return {
            child.GetName(): bool(self._is_shown(child))
            for child in parent.GetChildren()
            if child.GetName().startswith(self.DEFECT_PREFIX)
        }

    @staticmethod
    def _is_shown(prim) -> bool:
        from pxr import UsdGeom

        attr = UsdGeom.Imageable(prim).GetVisibilityAttr()
        return attr.Get() != UsdGeom.Tokens.invisible

    DEFECT_RENDER_STEPS = 8

    def show_defects(self, prop_name: str, visible: bool,
                     render_steps: Optional[int] = None) -> Dict[str, Any]:
        """Hide or show whatever marks a prop currently has, WITHOUT recomposing it.

        The round photographs a mark twice - once showing, once not - to prove the camera
        can actually see it, and that pair has to differ by the mark and nothing else.
        Switching the paint_defect VARIANT does that, and also recomposes the prim: measured,
        a variant switch mid-round left the next arm move failing with "Failed to get DOF
        velocities from backend", because recomposition briefly invalidates the physics
        views - including the arms'. Choosing which defect exists is a variant change and
        belongs at the start of a round; blinking it for a camera is a visibility change and
        belongs here.
        """
        self._require_booted()
        from pxr import UsdGeom

        stage = self._isaac.omni_usd.get_context().get_stage()
        parent = stage.GetPrimAtPath(f"/World/{_prim_name(prop_name)}")
        if not parent or not parent.IsValid():
            raise ValueError(f"no prop named {prop_name!r}")

        targets = []
        decals = stage.GetPrimAtPath(
            f"/World/{_prim_name(prop_name)}/Body/Decals")
        if decals and decals.IsValid():
            targets.append(decals)
        else:
            targets = [child for child in parent.GetChildren()
                       if child.GetName().startswith(self.DEFECT_PREFIX)]
        if not targets:
            raise ValueError(f"prop {prop_name!r} has no marks to show or hide")

        def _apply():
            for prim in targets:
                imageable = UsdGeom.Imageable(prim)
                if visible:
                    imageable.MakeVisible()
                else:
                    imageable.MakeInvisible()
            for _ in range(int(render_steps or self.DEFECT_RENDER_STEPS)):
                self.world.step(render=True)

        self.run(_apply)
        return {"showing": bool(visible),
                "prims": [p.GetPath().pathString for p in targets]}

    def set_defects(self, prop_name: str, wanted: Dict[str, bool],
                    render_steps: Optional[int] = None) -> Dict[str, bool]:
        """Show or hide a prop's defect marks, and do not return until it is drawn.

        Two things this has to get right.

        A mark is paint, so it is switched by visibility, and visibility is a render
        attribute - PhysX keeps the collider either way. That is fine here only because
        defect children are spawned with `collision: false`; it would not be fine for a
        structural child, and a "proud door" defect has to be modelled some other way.

        And the switch has to reach the screen. A toggle followed straight away by a
        frame grab hands back the render from before the toggle - a mislabelled training
        image, and the kind nobody notices. Measured: two render steps were not enough,
        and a caller that grabbed immediately got the old frame every time while the usd
        flag read back correctly, so the flag is no evidence.

        `render_steps` is how many frames to push through before returning. It is a
        number that was tuned rather than derived, so a caller that must be certain
        should also compare frames rather than trust this.
        """
        self._require_booted()
        from pxr import UsdGeom

        stage = self._isaac.omni_usd.get_context().get_stage()
        parent = stage.GetPrimAtPath(f"/World/{_prim_name(prop_name)}")
        if not parent or not parent.IsValid():
            raise ValueError(f"no prop named {prop_name!r}")

        painted = self._paint_variants(parent)
        if painted is not None:
            chosen, options = painted
            allowed = {f"{self.DEFECT_PREFIX}{name}": name for name in options}
            stray = set(wanted) - set(allowed)
            if stray:
                raise ValueError(
                    f"prop {prop_name!r} has no {sorted(stray)}; its paint defects are "
                    f"{sorted(n for n in options if n != self.NO_PAINT_DEFECT)}")
            on = sorted(name for name, want in wanted.items() if want)
            if len(on) > 1:
                raise ValueError(
                    f"prop {prop_name!r} carries its defects as a variant set, so it "
                    f"wears one at a time; asked for {on}")
            self.set_variants(
                prop_name,
                {self.PAINT_VARIANT_SET:
                 allowed[on[0]] if on else self.NO_PAINT_DEFECT},
                render_steps=render_steps)
            return self.defects(prop_name)

        known = {child.GetName(): child for child in parent.GetChildren()
                 if child.GetName().startswith(self.DEFECT_PREFIX)}
        unknown = set(wanted) - set(known)
        if unknown:
            raise ValueError(
                f"prop {prop_name!r} has no {sorted(unknown)}; its defects are "
                f"{sorted(known)}")

        steps = int(render_steps or self.DEFECT_RENDER_STEPS)

        def _apply():
            for defect, shown in wanted.items():
                imageable = UsdGeom.Imageable(known[defect])
                if shown:
                    imageable.MakeVisible()
                else:
                    imageable.MakeInvisible()
            # Push the change through to the render product the cameras read.
            for _ in range(steps):
                self.world.step(render=True)

        self.run(_apply)
        return self.defects(prop_name)

    def _strip_articulation_root(self, prim_path: str) -> None:
        """Remove ArticulationRootAPI from an asset's prims, keeping its joints."""
        from pxr import Usd, UsdPhysics

        stage = self._isaac.omni_usd.get_context().get_stage()
        root = stage.GetPrimAtPath(prim_path)
        if not root or not root.IsValid():
            raise ValueError(f"{prim_path}: not in the stage")
        removed = []
        for prim in Usd.PrimRange(root):
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
                removed.append(prim.GetPath().pathString)
        if not removed:
            LOGGER.info("prop %s: no articulation root to remove", prim_path)
        else:
            LOGGER.info("prop %s: articulation root removed from %s",
                        prim_path, removed)

    def place_prop(self, name: str, position_mm: Optional[List[float]] = None,
                   yaw_deg: Optional[float] = None,
                   orientation_wxyz: Optional[List[float]] = None) -> Dict[str, Any]:
        """Put a prop back where it belongs, at rest, WITHOUT tearing its joints apart.

        `reset` restores the props registered in `world.scene`, which the cube props are
        and a referenced asset is not - so a mailbox driven down a conveyor stayed where
        it finished, and the next run began from wherever the last one ended. That is not
        a small problem for a repeatability test: it silently turns three independent runs
        into one drifting sequence.

        The obvious implementation is wrong in a way worth recording. Writing the pose to
        the prop's `body_prim` moves the shell and leaves the hinged door where it was,
        which stretches the hinge across the cell and lets PhysX resolve it - the part
        left the scene at 20 metres. Every rigid body under the prop has to move as ONE
        body: the same translation, the same rotation about the same centre, and all
        velocities zeroed, or the part carries its old motion into its new pose.
        """
        self._require_booted()
        return self.run(lambda: self._place_one(name, position_mm, yaw_deg, orientation_wxyz))

    def _place_one(self, name: str, position_mm: Optional[List[float]] = None,
                   yaw_deg: Optional[float] = None,
                   orientation_wxyz: Optional[List[float]] = None) -> Dict[str, Any]:
        """The body of `place_prop`, callable from inside an already-scheduled callback."""
        import numpy as np
        from pxr import Usd, UsdPhysics

        configured = {str(p.get("name", "")): p for p in (self.cfg.props or [])}
        if name not in configured:
            raise ValueError(f"no prop {name!r}; have {sorted(configured)}")
        prop = configured[name]
        where = np.array([float(v) / 1000.0 for v in position_mm]
                         if position_mm is not None
                         else [float(v) for v in (prop.get("position") or (0, 0, 0))])
        yaw = float(prop.get("yaw_deg", 0.0)) if yaw_deg is None else float(yaw_deg)
        prim_path = f"/World/{_prim_name(name)}"
        body = prop.get("body_prim")
        anchor_path = f"{prim_path}/{str(body).lstrip('/')}" if body else prim_path

        def _place() -> Dict[str, Any]:

            stage = self._isaac.omni_usd.get_context().get_stage()
            root = stage.GetPrimAtPath(prim_path)
            if not root or not root.IsValid():
                raise ValueError(f"{prim_path}: not in the stage")
            bodies = [prim.GetPath().pathString for prim in Usd.PrimRange(root)
                      if prim.HasAPI(UsdPhysics.RigidBodyAPI)]
            # A KINEMATIC prop moves by its transform, not by its rigid view.
            #
            # `fixed` in this module means kinematic, not disabled, and a kinematic body
            # takes its pose from the usd xform on every physics step - so writing the
            # rigid view's world pose is overwritten before anything can read it. The call
            # still reported success ("bodies": 1) and the prop never moved: the belt
            # looked like it was indexing the shell from station to station and the shell
            # sat at x -55 the whole time, which is why the back-cap shot photographed
            # bare conveyor 338 mm away.
            #
            # Moving the root xform is also the RIGHT move for a kinematic prop, and it is
            # what the hinge-stretching note below is guarding against: children follow
            # their parent's transform, so the door cannot be left behind.
            if bodies and prop.get("fixed"):
                self._isaac.SingleXFormPrim(prim_path).set_world_pose(
                    position=where, orientation=_yaw_quat(yaw))
                return {"placed": name, "bodies": len(bodies), "kinematic": True,
                        "position_mm": [round(float(v) * 1000.0, 1) for v in where],
                        "yaw_deg": yaw}
            if not bodies:
                self._isaac.SingleXFormPrim(prim_path).set_world_pose(
                    position=where, orientation=_yaw_quat(yaw))
                return {"placed": name, "bodies": 0,
                        "position_mm": [round(float(v) * 1000.0, 1) for v in where]}

            handles, zeroed = {}, []
            for path in bodies:
                try:
                    handles[path] = self._isaac.SingleRigidPrim(path)
                except Exception:  # noqa: BLE001
                    LOGGER.exception("place_prop %s: no rigid view for %s", name, path)
            if anchor_path not in handles:
                anchor_path_local = bodies[0]
            else:
                anchor_path_local = anchor_path
            poses = {path: handle.get_world_pose() for path, handle in handles.items()}
            anchor_pos, anchor_quat = poses[anchor_path_local]
            # The rigid rotation that takes the prop from where it is to the yaw asked
            # for, applied about the anchor so relative geometry is untouched.
            turn = _quat_mul(_yaw_quat(yaw), _quat_conj(_yaw_of(anchor_quat)))
            # A FULL orientation, when one is asked for. Yaw-only keeps whatever roll the
            # part had, and a blade standing in a feeder slot that a reset left rolled 6
            # degrees is placed already pressed into the slot, where it jams.
            if orientation_wxyz is not None:
                want = np.asarray([float(v) for v in orientation_wxyz])
                turn = _quat_mul(want / np.linalg.norm(want),
                                 _quat_conj(np.asarray(anchor_quat)))
            for path, handle in handles.items():
                pos, quat = poses[path]
                offset = _quat_rotate(turn, np.asarray(pos) - np.asarray(anchor_pos))
                handle.set_world_pose(position=where + offset,
                                      orientation=_quat_mul(turn, np.asarray(quat)))
                for setter in ("set_linear_velocity", "set_angular_velocity"):
                    fn = getattr(handle, setter, None)
                    if fn is None:
                        zeroed.append(f"{setter}:missing")
                        continue
                    try:
                        fn(np.array([0.0, 0.0, 0.0]))
                        zeroed.append(f"{setter}:ok")
                    except Exception as exc:  # noqa: BLE001
                        zeroed.append(f"{setter}:{type(exc).__name__}")
                        LOGGER.exception("%s: could not zero %s", name, setter)
            # Report whether the velocities were actually zeroed.
            #
            # They were being zeroed "best effort" through getattr, so a missing setter was
            # indistinguishable from a successful one - and a dynamic part teleported across
            # the cell with its old velocity intact leaves at kilometres per second, which
            # is exactly what this cell kept seeing. Silence is not success.
            return {"placed": name, "bodies": len(handles),
                    "position_mm": [round(float(v) * 1000.0, 1) for v in where],
                    "zeroed": zeroed, "yaw_deg": yaw}

        return _place()

    def join(self, name: str, body0: str, body1: str, kind: str = "fixed",
             anchor_mm: Optional[List[float]] = None,
             axis: Optional[List[float]] = None,
             limits_deg: Optional[List[float]] = None,
             local0_mm: Optional[List[float]] = None,
             local1_mm: Optional[List[float]] = None,
             drive: Optional[Dict[str, Any]] = None,
             release_kinematic: bool = False) -> Dict[str, Any]:
        """Fasten one body to another, the way assembling them would.

        `release_kinematic` makes body1 dynamic again IN THE SAME CALL, before the joint is
        authored and with no physics step between: a pin pressed home kinematically and
        jointed first, then released, drifted 7 mm across its bore - the toggle rebuilt the
        body and the joint made before it did not hold (2026-10-13).

        Parts do not stay assembled because they are in the right place. The mailbox's flag
        measured that for us: 2.7 mm of wall with 1.00 mm of radial slack lets the peg tilt
        36 degrees against a 140 mm lever, so the arm placed it to 0.78 mm and it backed out
        and fell the moment the cup let go. The asset's answer is that SEATING creates a
        joint - a snap or rivet for the flag, a press fit for the pin, a hinge for the door -
        and it publishes the kind, the anchor and, for the hinge, the axis and the limits.
        Making that joint is the cell's job, and until now the cell had no way to.

        The anchor is given in WORLD millimetres and resolved into each body's own frame, so
        the caller states where the fastener IS rather than where it is relative to two
        different parts' origins. Re-joining under the same name replaces the joint, which
        is what a cell that runs a cycle twice needs.
        """
        self._require_booted()
        kind = str(kind).lower()
        if kind not in ("fixed", "revolute"):
            raise ValueError(f"join: kind must be fixed or revolute, not {kind!r}")

        def _join() -> Dict[str, Any]:
            from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

            stage = self._isaac.omni_usd.get_context().get_stage()
            if release_kinematic and stage.GetPrimAtPath(body1).IsValid():
                for prim in Usd.PrimRange(stage.GetPrimAtPath(body1)):
                    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                        UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(False)
            for path in (body0, body1):
                if not stage.GetPrimAtPath(path).IsValid():
                    raise ValueError(f"join: {path}: not in the stage")
            UsdGeom.Scope.Define(stage, JOINTS_PRIM_PATH)
            path = f"{JOINTS_PRIM_PATH}/{_prim_name(name)}"
            if stage.GetPrimAtPath(path).IsValid():
                stage.RemovePrim(path)
            joint = (UsdPhysics.RevoluteJoint.Define(stage, path) if kind == "revolute"
                     else UsdPhysics.FixedJoint.Define(stage, path))
            joint.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
            joint.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
            # The joint's frames, PUBLISHED where the asset publishes them.
            #
            # Deriving them from where the bodies happen to be at the moment of joining
            # freezes in whatever the grasp got wrong. Measured: a suction cup held the
            # flag about 2 degrees out of square, the joint was built around that, and
            # enforcing it then dragged the part 8 mm back out of the hole - a joint doing
            # exactly what it was told, to the wrong pose. Given explicit local frames the
            # joint instead PULLS the part square, which is what a snap fit physically
            # does. The world anchor remains for the case where nothing is published.
            if local0_mm is not None and local1_mm is not None:
                locals_ = [Gf.Vec3d(*[float(v) / 1000.0 for v in local0_mm]),
                           Gf.Vec3d(*[float(v) / 1000.0 for v in local1_mm])]
            else:
                anchor = Gf.Vec3d(*[float(v) / 1000.0
                                    for v in (anchor_mm or (0, 0, 0))])
                locals_ = []
                for body in (body0, body1):
                    xf = UsdGeom.Xformable(stage.GetPrimAtPath(body))
                    world = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                    locals_.append(world.GetInverse().Transform(anchor))
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*locals_[0]))
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*locals_[1]))
            joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
            joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
            if kind == "revolute":
                # UsdPhysics names a revolute axis by letter, so the published vector picks
                # the letter it points along. The bodies are axis aligned when seated -
                # which is what makes the identity rotations above correct - so this is a
                # lookup rather than a decomposition.
                vec = [abs(float(v)) for v in (axis or (0.0, 0.0, 1.0))]
                joint.CreateAxisAttr("XYZ"[vec.index(max(vec))])
                if limits_deg:
                    joint.CreateLowerLimitAttr(float(limits_deg[0]))
                    joint.CreateUpperLimitAttr(float(limits_deg[1]))
                if drive:
                    # A hinge without a spring is just a hinge, and for this door that is
                    # not a detail. Its centre of mass sits 112.7 mm straight above the
                    # hinge axis with a 1.5 mm horizontal offset, so a plain revolute makes
                    # it an inverted pendulum: measured, it swung to its 100 degree limit
                    # and hung there, 305 mm from where it is supposed to be. The spring is
                    # what holds it shut, and it has to be modelled rather than assumed.
                    api = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
                    api.CreateTypeAttr("force")
                    api.CreateStiffnessAttr(float(drive.get("stiffness", 0.0)))
                    api.CreateDampingAttr(float(drive.get("damping", 0.0)))
                    api.CreateTargetPositionAttr(float(drive.get("target_deg", 0.0)))
                    if drive.get("max_force") is not None:
                        api.CreateMaxForceAttr(float(drive["max_force"]))
            LOGGER.info("joined %s to %s at %s as %s (%s)", body1, body0, anchor_mm,
                        kind, path)
            return {"joint": path, "kind": kind, "drive": bool(drive),
                    "local0_mm": [round(float(v) * 1000.0, 2) for v in locals_[0]],
                    "local1_mm": [round(float(v) * 1000.0, 2) for v in locals_[1]]}

        return self.run(_join)

    def unjoin(self, name: str) -> Dict[str, Any]:
        """Undo a `join`, and have it take effect NOW rather than at the next reset.

        Removing the prim is not enough and never was. PhysX builds its articulation from
        the stage when the scene is parsed, so a joint whose prim has gone on is still a
        live constraint until something re-reads - and `reset` was the only thing that
        did. Everything built on top of that inherited the lag: a part unjoined and then
        placed is a body still fixed to a 40 kg pallet being teleported across the cell,
        measured at z 211219, 255148, 267731 and 271096 on four consecutive runs; and a
        clip re-authored with a corrected anchor simply did nothing, leaving the carried
        part 5.4 mm off its slot with the log saying "the clip is not pulling it back".

        `physics:jointEnabled` is the attribute PhysX watches, so clearing it releases the
        constraint on the next step. The prim still goes, so a later `join` of the same
        name builds a clean one; disabling first is what makes the removal reach the
        simulator in the same breath.
        """
        self._require_booted()

        def _unjoin() -> Dict[str, Any]:
            from pxr import UsdPhysics

            stage = self._isaac.omni_usd.get_context().get_stage()
            path = f"{JOINTS_PRIM_PATH}/{_prim_name(name)}"
            prim = stage.GetPrimAtPath(path)
            existed = prim.IsValid()
            if existed:
                joint = UsdPhysics.Joint(prim)
                if joint:
                    joint.CreateJointEnabledAttr().Set(False)
                stage.RemovePrim(path)
            return {"removed": existed, "joint": path}

        return self.run(_unjoin)

    JAW_PRIM = "jaw"

    def author_jaw(self, parent_prim: str, spec: Dict[str, Any]) -> Dict[str, Any]:
        """Give a gripper something you can SEE, and something that moves when it grips.

        The surface gripper authors no geometry at all: it is an attachment point and a
        fabricated joint, so on film a part simply sticks to the end of a bare flange. That
        is not a rendering quibble - a cell whose grasping is invisible cannot be reviewed
        by watching it, and watching it is how three of this build's worst bugs were found.

        So: a body on the flange and two fingers reaching down the tool axis, which close
        when the gripper closes. The fingers are KINEMATIC - the hold is still the
        fabricated joint, which is the mechanism that was measured and works - and they are
        placed where that joint attaches, so what you see is where the part is held rather
        than a decoration near it.
        """
        self._require_booted()
        if spec.get("usd_path"):
            return self._author_tool_asset(parent_prim, spec)
        reach = float(spec.get("reach_mm", 120.0))
        body = float(spec.get("body_mm", 72.0))
        finger = [float(v) for v in spec.get("finger_mm", (14.0, 18.0))]
        colour = [float(v) for v in spec.get("color", (0.26, 0.28, 0.32))]

        def _author() -> Dict[str, Any]:
            from pxr import Gf, UsdGeom

            stage = self._isaac.omni_usd.get_context().get_stage()
            self._require_prim(parent_prim)
            root = f"{parent_prim}/{self.JAW_PRIM}"
            if stage.GetPrimAtPath(root).IsValid():
                stage.RemovePrim(root)
            UsdGeom.Xform.Define(stage, root)

            def block(name, pos, size, col):
                cube = UsdGeom.Cube.Define(stage, f"{root}/{name}")
                cube.CreateSizeAttr(1.0)
                cube.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5),
                                       Gf.Vec3f(0.5, 0.5, 0.5)])
                x = UsdGeom.Xformable(cube.GetPrim())
                x.ClearXformOpOrder()
                x.AddTranslateOp().Set(Gf.Vec3d(*[v / 1000.0 for v in pos]))
                x.AddScaleOp().Set(Gf.Vec3f(*[v / 1000.0 for v in size]))
                cube.CreateDisplayColorAttr([Gf.Vec3f(*col)])
                return cube

            # The tool runs along the flange's +x (the gripper docstring is the authority),
            # so the body sits just off the flange and the fingers reach out along it.
            block("body", (body / 2.0, 0.0, 0.0), (body, body * 0.8, body * 0.8), colour)
            for sign in (-1.0, 1.0):
                block(f"finger_{'my' if sign < 0 else 'py'}",
                      (body + (reach - body) / 2.0, 0.0, sign * 0.0),
                      (reach - body, finger[0], finger[1]),
                      [c * 1.5 for c in colour])
            return {"jaw": root, "reach_mm": reach}

        out = self.run(_author)
        self._jaw_specs[parent_prim] = dict(spec)
        self.set_jaw(parent_prim, closed=False)
        return out

    def _author_tool_asset(self, parent_prim: str, spec: Dict[str, Any]) -> Dict[str, Any]:
        """Hang a REAL gripper off the flange, instead of two boxes that stand in for one.

        Two boxes were honest about one thing - that something is holding the part - and
        dishonest about everything else, and a course that explains a Hand-E over a picture
        of a cuboid teaches the cuboid. So the asset goes in: Robotiq's own Hand-E for the
        parts arm, Universal Robots' own suction unit for the handling arm.

        VISUAL ONLY, and that is the whole subtlety. Both assets ship as working robots -
        the Hand-E carries a PhysicsArticulationRootAPI with two prismatic joints and three
        rigid bodies, the suction unit is a rigid body with mass and a compliant cup joint.
        Referenced under a UR's flange as they are, the Hand-E's articulation nests inside
        the arm's, which PhysX does not allow, and the suction unit's body hangs off a link
        it is not jointed to. So every physics API under the tool is stripped and the hold
        stays what it already was: the surface gripper's fabricated joint, which is the
        mechanism that was measured and works.

        What remains is geometry that moves: the finger prims are driven directly, the way
        the cube fingers were, so a closing gripper still looks like one.
        """
        import math

        from pxr import Gf, Usd, UsdGeom, UsdPhysics, PhysxSchema

        usd_path = str(spec["usd_path"])
        if self._usd_exists(usd_path) is False:
            raise ValueError(f"gripper tool: usd not found: {usd_path}")
        axis = [float(v) for v in (spec.get("axis") or (1.0, 0.0, 0.0))]
        mount_mm = float(spec.get("mount_mm", 0.0))

        def _author() -> Dict[str, Any]:
            stage = self._isaac.omni_usd.get_context().get_stage()
            self._require_prim(parent_prim)
            # A second body on the same flange (arm-b's RealSense beside its EZU) gets its
            # own prim name: under `jaw` it would replace the gripper it is bolted to.
            root = f"{parent_prim}/{spec.get('prim_name') or self.JAW_PRIM}"
            if stage.GetPrimAtPath(root).IsValid():
                stage.RemovePrim(root)
            self._isaac.add_reference_to_stage(usd_path=usd_path, prim_path=root)
            # A tool's variants, e.g. the fisheye tool's {"head": "side40"} (Sam): the
            # lens, ring flash and colliders turned with the camera mounted off the axis.
            for vset, choice in (spec.get("variants") or {}).items():
                sets = stage.GetPrimAtPath(root).GetVariantSets()
                if not sets.HasVariantSet(str(vset)):
                    raise ValueError(f"tool {usd_path}: no variant set {vset!r}")
                sets.GetVariantSet(str(vset)).SetVariantSelection(str(choice))

            # Point the asset's own tool axis down the flange's +x, which is where this
            # module's grippers look, then slide it so its MOUNTING FACE sits on the
            # flange rather than its origin.
            from .spatial import quat_axis_onto_x
            qw, qx, qy, qz = quat_axis_onto_x(tuple(axis))
            rot = Gf.Rotation(Gf.Quatd(qw, Gf.Vec3d(qx, qy, qz)))
            x = UsdGeom.Xformable(stage.GetPrimAtPath(root))
            x.ClearXformOpOrder()
            x.AddTranslateOp().Set(Gf.Vec3d(mount_mm / 1000.0, 0.0, 0.0))
            # Matching the precision the asset authored: the EZU's root carries a quatd
            # orient, and an op of the same name in the other precision is refused.
            have = x.GetPrim().GetAttribute("xformOp:orient")
            if have and str(have.GetTypeName()) == "quatd":
                x.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(rot.GetQuat()))
            else:
                x.AddOrientOp().Set(Gf.Quatf(rot.GetQuat()))
            # Clearing the op order above also clears whatever the asset's DEFAULT PRIM
            # authored, because a reference brings the default prim in as this prim. The
            # Schunk EZU's root carries a 90 degree X turn and a 0.001 scale (it is
            # modelled Y-up in mm), so without restating the scale it arrives 1000x size.
            # `axis` is then given in that unscaled, unturned frame.
            if spec.get("scale") is not None:
                k = float(spec["scale"])
                had = x.GetPrim().GetAttribute("xformOp:scale")
                if had and str(had.GetTypeName()) == "double3":
                    x.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(k, k, k))
                else:
                    x.AddScaleOp().Set(Gf.Vec3f(k, k, k))

            # Strip the physics. Everything: articulation root, rigid bodies, mass, joints
            # and colliders. A collider on a finger fights the fabricated joint that is
            # actually holding the part, and an articulation inside an articulation is an
            # error PhysX reports once and then behaves unpredictably about.
            stripped = {"articulation": 0, "body": 0, "collider": 0, "joint": 0}
            # De-instance first. A PrimRange does not enter an instance, and the EZU keeps
            # 13 of its 16 colliders inside instanced `collisions` scopes: stripped without
            # this, they rode on the wrist and stalled arm-b against the pin nest.
            while True:
                inst = [p for p in Usd.PrimRange(stage.GetPrimAtPath(root))
                        if p.IsInstanceable()]
                if not inst:
                    break
                for prim in inst:
                    prim.SetInstanceable(False)
            for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
                if prim.IsA(UsdPhysics.Joint):
                    prim.SetActive(False)
                    stripped["joint"] += 1
                    continue
                for api, key in ((UsdPhysics.ArticulationRootAPI, "articulation"),
                                 (UsdPhysics.RigidBodyAPI, "body"),
                                 (UsdPhysics.MassAPI, "body"),
                                 (UsdPhysics.CollisionAPI, "collider")):
                    if prim.HasAPI(api):
                        prim.RemoveAPI(api)
                        stripped[key] += 1
                for api in (PhysxSchema.PhysxArticulationAPI,
                            PhysxSchema.PhysxRigidBodyAPI,
                            PhysxSchema.PhysxCollisionAPI):
                    if prim.HasAPI(api):
                        prim.RemoveAPI(api)
            # ATTACHMENTS: further visual-only assets hung under the tool's own prims - the
            # EZU's pin inserts, each referenced under the finger it is screwed to so it
            # rides that finger's slide. [{"usd_path", "prim", "under"}], `under` relative
            # to the tool's root.
            for k, att in enumerate(spec.get("attachments") or []):
                parent = stage.GetPrimAtPath(f"{root}/{att['under']}")
                if not parent or not parent.IsValid():
                    LOGGER.warning("tool %s: no prim %s to attach %s under", root,
                                   att["under"], att.get("prim"))
                    continue
                at = f"{parent.GetPath()}/attachment_{k}"
                held = stage.DefinePrim(at, "Xform")
                held.GetReferences().AddReference(str(att["usd_path"]),
                                                  str(att.get("prim") or ""))
                stripped["attachments"] = stripped.get("attachments", 0) + 1
            # The tool's OWN light, where it publishes one: the fisheye tool carries a ring
            # flash (`flash:light` -> a DiskLight, authored dark, with the intensity to
            # drive it at in `flash:onIntensity`). Nothing drove it, so the interior was lit
            # by a stand-in sphere while the real flash sat in the model unlit.
            tool_prim = stage.GetPrimAtPath(root)
            rel = tool_prim.GetRelationship("flash:light")
            on = tool_prim.GetAttribute("flash:onIntensity")
            which = tool_prim.GetAttribute("flash:attribute")
            if rel and rel.GetTargets() and on and on.HasValue():
                name = str(which.Get()) if which and which.HasValue() else "inputs:intensity"
                for target in rel.GetTargets():
                    light = stage.GetPrimAtPath(target)
                    if light and light.IsValid() and light.GetAttribute(name):
                        light.GetAttribute(name).Set(float(on.Get()))
                        LOGGER.info("tool %s: flash %s on at %s", parent_prim, target,
                                    float(on.Get()))
            LOGGER.info("gripper tool %s: referenced %s, stripped %s",
                        parent_prim, usd_path.rsplit("/", 1)[-1], stripped)
            return {"jaw": root, "tool": usd_path.rsplit("/", 1)[-1],
                    "stripped": stripped}

        out = self.run(_author)
        if spec.get("prim_name"):
            return out            # a body, not a gripper: nothing to open or close
        self._jaw_specs[parent_prim] = dict(spec)
        self.set_jaw(parent_prim, closed=False)
        return out

    def set_jaw(self, parent_prim: str, closed: bool,
                gap_mm: Optional[float] = None) -> Dict[str, Any]:
        """Open or close the visible fingers.

        `gap_mm` overrides the configured opening for this one move, which is how a close
        lands on the actual part rather than on a number that suits one of two parts.

        `open_mm` and `closed_mm` are the CLEAR OPENING between the finger faces, which is
        what a gripper's spec quotes and what the config means by them. The first version
        used them as the centre-to-centre pitch and a finger has thickness, so the two
        passed THROUGH each other: 26 mm fingers at a 10 mm pitch sit at +-5 mm with their
        inner faces at -+8, overlapping by 16 mm. On film that is a gripper that closes
        into itself and a part that sticks to nothing in particular - which is the whole
        complaint the visible jaws were added to answer.
        """
        spec = getattr(self, "_jaw_specs", {}).get(parent_prim)
        if spec is None:
            return {"jaw": None}
        if spec.get("usd_path"):
            return self._set_tool_fingers(parent_prim, spec, closed, gap_mm)
        thick = float((spec.get("finger_mm") or (14.0, 18.0))[1])
        want = (float(spec.get("closed_mm", 10.0)) if closed
                else float(spec.get("open_mm", 62.0)))
        if closed and gap_mm is not None:
            # Never wider than the jaw opens, and never negative: a part wider than the
            # gripper is a teaching fault to see, not one to hide by splaying the fingers.
            want = max(0.0, min(float(gap_mm), float(spec.get("open_mm", 62.0))))
        gap = (want + thick) / 2.0

        def _move() -> Dict[str, Any]:
            from pxr import Gf, UsdGeom

            stage = self._isaac.omni_usd.get_context().get_stage()
            for sign, name in ((-1.0, "finger_my"), (1.0, "finger_py")):
                prim = stage.GetPrimAtPath(
                    f"{parent_prim}/{self.JAW_PRIM}/{name}")
                if not prim.IsValid():
                    continue
                ops = UsdGeom.Xformable(prim).GetOrderedXformOps()
                for op in ops:
                    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                        at = op.Get()
                        op.Set(Gf.Vec3d(at[0], at[1], sign * gap / 1000.0))
            return {"jaw": parent_prim, "closed": closed,
                    # The clear opening, not the pitch - the same thing the config asked
                    # for, so a caller can check it got what it set.
                    "gap_mm": round(gap * 2.0 - thick, 1)}

        return self.run(_move)

    def _set_tool_fingers(self, parent_prim: str, spec: Dict[str, Any],
                          closed: bool, gap_mm: Optional[float]) -> Dict[str, Any]:
        """Slide a referenced gripper's own finger prims to an opening.

        The asset's prismatic joints are switched off - it is visual geometry under an
        arm's articulation, not a robot of its own - so the fingers are translated here
        instead, along the slide direction the asset's joints declare. `fingers` maps each
        finger prim to the unit direction, in the asset's own frame, that OPENS it.
        """
        from pxr import Gf, UsdGeom

        fingers = dict(spec.get("fingers") or {})
        if not fingers:
            return {"jaw": None, "why": "the tool declares no finger prims"}
        stroke = float(spec.get("stroke_mm", 25.0))
        full = float(spec.get("open_mm", stroke * 2.0))
        want = float(spec.get("closed_mm", 0.0)) if closed else full
        if closed and gap_mm is not None:
            want = max(0.0, min(float(gap_mm), full))
        # Each finger travels half the opening, from the closed position outwards.
        #
        # Unless the asset's rest pose is NOT closed. The EZU's jaws rest with a 26 mm
        # clear opening, so the travel is measured from there and is negative to close on
        # anything narrower; `rest_open_mm` says what the rest opening is.
        rest_open = float(spec.get("rest_open_mm", 0.0))
        # Inserts on the finger tips (the EZU's pin inserts stand `insert_mm` proud of the
        # pads) close on the part, so the PADS stop that much wider each side - and never
        # closer than where the flat pads meet each other (`min_open_mm`).
        want += 2.0 * float(spec.get("insert_mm", 0.0))
        floor_open = float(spec.get("min_open_mm", 0.0))
        travel = max((floor_open - rest_open) / 2.0, min((want - rest_open) / 2.0, stroke))
        # And the translate ops are in the PARENT's units, which for a referenced tool is
        # the asset's own: metres for the Hand-E, millimetres under the EZU's 0.001 scale.
        per_mm = float(spec.get("units_per_mm", 0.001))

        def _move() -> Dict[str, Any]:
            stage = self._isaac.omni_usd.get_context().get_stage()
            root = f"{parent_prim}/{self.JAW_PRIM}"
            moved = []
            for name, direction in fingers.items():
                prim = stage.GetPrimAtPath(f"{root}/{name}")
                if not prim.IsValid():
                    continue
                d = [float(v) for v in direction]
                x = UsdGeom.Xformable(prim)
                ops = [o for o in x.GetOrderedXformOps()
                       if o.GetOpType() == UsdGeom.XformOp.TypeTranslate]
                # The asset's own rest translate is the datum; the opening is added to it,
                # so a finger returns exactly to where the asset put it when closed to 0.
                rest = self._tool_rest.setdefault(f"{root}/{name}",
                                                  list(ops[0].Get()) if ops else [0, 0, 0])
                at = [rest[i] + d[i] * travel * per_mm for i in range(3)]
                if ops:
                    ops[0].Set(Gf.Vec3d(*at))
                else:
                    x.AddTranslateOp().Set(Gf.Vec3d(*at))
                moved.append(name)
            return {"jaw": parent_prim, "closed": closed, "gap_mm": round(want, 1),
                    "fingers": moved}

        return self.run(_move)

    def set_translate(self, prim_path: str, translate_mm: List[float]) -> Dict[str, Any]:
        """Set a prim's local `xformOp:translate`, in mm - for a mechanism an asset publishes
        as a moving Xform, such as the pin magazine's lifter (`magazine:lifter`). Its
        collider follows the transform, so a part standing on it is carried."""
        self._require_booted()
        from pxr import Gf, UsdGeom

        def _set():
            stage = self._isaac.omni_usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                raise ValueError(f"{prim_path}: not in the stage")
            xf = UsdGeom.Xformable(prim)
            op = next((o for o in xf.GetOrderedXformOps()
                       if o.GetOpType() == UsdGeom.XformOp.TypeTranslate), None)
            if op is None:
                op = xf.AddTranslateOp()
            v = [float(t) / 1000.0 for t in translate_mm]
            if op.GetPrecision() == UsdGeom.XformOp.PrecisionDouble:
                op.Set(Gf.Vec3d(*v))
            else:
                op.Set(Gf.Vec3f(*v))
            return {"path": prim_path, "translate_mm": list(translate_mm)}

        return self.run(_set)

    def place_props(self, placements: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Move several props so they land in the SAME physics step.

        One call per prop is one step per prop, and that is the whole problem once anything
        on the fixture is dynamic: the saddle arrives at the next station while the shell
        standing in it is still at the last one, and physx resolves the penetration the only
        way it can. Measured, the assembled mailbox left the cell at 423 metres, and again
        at 2.6 km when the move was broken into 70 mm steps - smaller jumps, same fault,
        because the fault is not the size of the jump but that the set is not moved
        together.

        Inside one scheduled callback the whole set moves before anything is simulated, so
        the relative geometry never goes wrong in the first place.
        """
        self._require_booted()
        items = [(str(p.get("name", "")), p.get("position_mm"),
                  None if p.get("yaw_deg") is None else float(p["yaw_deg"]))
                 for p in placements]

        def _place_all() -> Dict[str, Any]:
            return {"placed": [self._place_one(n, pos, yaw) for n, pos, yaw in items]}

        return self.run(_place_all)

    def set_door(self, prop: str = "part", degrees: float = 0.0,
                 body: str = "Door", axis: str = "x") -> Dict[str, Any]:
        """Stage the part's door at an angle, by placing its body on the hinge's arc.

        Opening the door with an arm is the hard way round, and it is the wrong way round
        for the check. Released from open, this asset's four conditions tell themselves
        apart with no arm force at all: `ok` shuts itself (its spring makes 1.50 N.m
        against gravity's 0.47), `loose` has no spring and stays where gravity leaves it,
        `binding` shuts under the same spring through 83x the damping, and `stuck` cannot
        be staged open past its 3 degree limit - which is itself the reading.

        So the cell stages the door open and WATCHES. The arm's job becomes closing
        whatever did not close, and a closing push presses the part down into its nest
        rather than levering it out, which is the direction the fixture can take.

        The door body's origin IS the hinge (the joint is authored `localPos1 = (0,0,0)`
        on it), so staging is a rotation of that body about its own origin - no joint
        drive to fight, and physics takes over from there.
        """
        import numpy as np

        self._require_booted()
        prim_path = f"/World/{_prim_name(prop)}/{body}"

        def _place() -> Dict[str, Any]:
            handle = self._isaac.SingleRigidPrim(prim_path)
            position, quat = handle.get_world_pose()
            shut = _yaw_quat(0.0)
            # The hinge turns about the body's own x in world terms once the part is
            # yawed; rotate the CURRENT orientation by the wanted angle about it.
            half = math.radians(degrees) / 2.0
            unit = {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0),
                    "z": (0.0, 0.0, 1.0)}[axis]
            turn = np.array([math.cos(half)] + [math.sin(half) * v for v in unit])
            handle.set_world_pose(position=np.asarray(position),
                                  orientation=_quat_mul(turn, np.asarray(quat)))
            for setter in ("set_linear_velocity", "set_angular_velocity"):
                fn = getattr(handle, setter, None)
                if fn is not None:
                    fn(np.array([0.0, 0.0, 0.0]))
            _ = shut
            return {"staged": prim_path, "degrees": float(degrees)}

        return self.run(_place)

    def _setup_conveyor(self, name: str, prim_path: str, prop: Dict[str, Any]) -> None:
        """Make a conveyor's rollers drivable, and arm its break beam.

        PhysX moves a part along a conveyor by the SURFACE velocity of a kinematic rigid
        body - the body itself never moves, but contact is solved as though its surface
        were sliding. That is how `ConveyorBeltDemo` and the `isaacsim.asset.gen.conveyor`
        node both work, and it is why `fixed` had to mean kinematic rather than disabled.

        A05's rollers ship as plain colliders with no rigid body at all, so the body is
        applied here. Each roller is a cylinder whose axis runs along the asset's y, which
        the prop's -90 degree yaw puts along world x, so an angular velocity about x moves
        the top of the roller along y.

        USD states angular velocity in DEGREES per second, which is the kind of unit that
        is wrong by a factor of 57 without ever looking wrong.
        """
        from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

        cfg = prop["conveyor"]
        stage = self._isaac.omni_usd.get_context().get_stage()
        # `driven` names the scope of geometry to drive, under the prop's prim - which is
        # how a conveyor ASSET is built, with its deck or its rollers in a named child. A
        # cube prop has no children and does not need any: a flat deck is a flat deck, and
        # the prop's own prim is the geometry. "." says so, rather than requiring a
        # 4.6 metre belt to be modelled as a usd asset before it can be driven.
        driven = str(cfg.get("driven", "Rollers"))
        path = prim_path if driven in (".", "") else f"{prim_path}/{driven}"
        scope = stage.GetPrimAtPath(path)
        if not scope or not scope.IsValid():
            raise ValueError(f"conveyor {name}: no driven scope at {path}")
        cache = self._bbox_cache()
        xform = UsdGeom.XformCache()
        rollers, radius = [], 0.0
        for prim in Usd.PrimRange(scope):
            if not prim.IsA(UsdGeom.Gprim):
                continue
            body = UsdPhysics.RigidBodyAPI.Apply(prim)
            body.CreateKinematicEnabledAttr().Set(True)
            PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            body.CreateAngularVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            body.CreateVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            if str(cfg.get("mode", "rollers")) == "belt":
                if not hasattr(PhysxSchema, "PhysxSurfaceVelocityAPI"):
                    raise ValueError(
                        f"conveyor {name}: mode 'belt' needs PhysxSurfaceVelocityAPI, "
                        f"which this build of PhysxSchema does not have")
                PhysxSchema.PhysxSurfaceVelocityAPI.Apply(prim)
            # Angular velocity is applied in the BODY's frame, and this asset's rollers
            # are modelled in two orientations 180 degrees apart - 14 one way, 15 the
            # other. One world-space spin set on all of them therefore drives alternate
            # rollers in OPPOSITE directions: a tug of war whose winner depends on which
            # rollers the part happens to be touching. The part crawled, stalled, and
            # twice set off upstream, all from the same config. Each roller's own
            # rotation is kept here so the drive can be expressed in its frame.
            quat = xform.GetLocalToWorldTransform(prim).ExtractRotationQuat()
            rollers.append({
                "path": prim.GetPath().pathString,
                "quat": [float(quat.GetReal())] + [float(v) for v in quat.GetImaginary()],
            })
            extent = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            radius = max(radius, (extent.GetMax()[2] - extent.GetMin()[2]) / 2.0)
        if not rollers:
            raise ValueError(f"conveyor {name}: driven scope holds no geometry")
        beam = cfg.get("beam") or {}
        self._conveyors[name] = {
            "mode": str(cfg.get("mode", "rollers")),
            "prim_path": prim_path,
            "rollers": rollers,
            "radius_mm": radius * 1000.0,
            "speed_mm_s": float(cfg.get("speed_mm_s", 300.0)),
            # Which world direction a positive speed carries a part. Defaults to +y,
            # which is the infeed's; the outfeeds run along x and say so.
            "axis": tuple(float(v) for v in (cfg.get("axis") or (0.0, 1.0, 0.0))),
            "running": False,
            "beam_y_mm": float(beam.get("y_mm", 0.0)),
            "beam_from_x_mm": float(beam.get("from_x_mm", 0.0)),
            "beam_span_mm": float(beam.get("span_mm", 0.0)),
            "beam_above_deck_mm": float(beam.get("above_deck_mm", 30.0)),
            "beam_z_mm": float(beam.get("z_mm", 0.0)),
            # Normally derived from the belt's own axis; stated only when a beam is
            # deliberately not square to the travel.
            "beam_along": (tuple(float(v) for v in beam["along"])
                           if beam.get("along") else None),
            "blocked": False,
            # ARMED ONLY IF THERE IS A BEAM TO ARM.
            #
            # This defaulted to True, and a conveyor with no `beam` block then had one
            # anyway: span 0, at the world origin. `_beam_blocked` raycast it every
            # physics step, PhysX rejected it - "NpSceneQueries::multiQuery input check:
            # distance cannot be negative or zero", thousands of them in the log - and the
            # degenerate query read as a hit, so the belt stopped on the first step after
            # every start. Measured on the assembly cell: twelve indexes in a row reported
            # "pallet at -55, +207 mm to go" with the pallet having moved exactly 0 mm,
            # which reads as a friction or a stiction problem and is neither.
            #
            # A beam that spans nothing is not a beam. `stops` can still be set explicitly
            # either way; what it cannot do any more is arm itself.
            "stops": bool(beam.get("stops", bool(beam)))
            and float(beam.get("span_mm", 0.0)) > 0.0,
        }
        LOGGER.info("conveyor %s: %d rollers, radius %.1f mm",
                    name, len(rollers), radius * 1000.0)
        self.world.add_physics_callback(f"conveyor_{name}", self._conveyor_step)

    def _conveyor_step(self, _step_size: float) -> None:
        """Watch each armed beam and stop its conveyor the step the beam breaks.

        This has to live inside the sim. A client polling the sensor over gRPC and then
        commanding a stop adds tens of milliseconds of jitter, and at 300 mm/s that alone
        is more than T2's 10 mm acceptance.
        """
        for name, conveyor in self._conveyors.items():
            if not conveyor["running"] or not conveyor["stops"]:
                continue
            if self._beam_blocked(conveyor):
                conveyor["blocked"] = True
                self._drive_rollers(conveyor, 0.0)
                conveyor["running"] = False

    def _beam_blocked(self, conveyor: Dict[str, Any]) -> bool:
        """Is anything across the beam right now?

        A non-positive span is not a question PhysX can answer: a zero-distance scene
        query is rejected, and the rejection came back looking like a hit. Say "nothing is
        across it" rather than ask.
        """
        span = float(conveyor.get("beam_span_mm", 0.0))
        if span <= 0.0:
            return False
        # ACROSS the belt, not along it.
        #
        # The direction was hard-coded +X, which was right for the cell this came from -
        # its infeed runs along y, so a beam across it points along x. This belt runs
        # along X, so the same hard-coded ray fires straight down the middle of the deck
        # and reports the first thing anywhere on it, which is not a break beam, it is a
        # proximity sensor pointed the wrong way. A conveyor that declares which way it
        # carries a part already knows which way is across it.
        along = conveyor.get("axis") or (1.0, 0.0, 0.0)
        across = conveyor.get("beam_along") or (-float(along[1]), float(along[0]), 0.0)
        hit = self._raycast_raw(
            [conveyor["beam_from_x_mm"], conveyor["beam_y_mm"], conveyor["beam_z_mm"]],
            [float(v) for v in across], span)
        return bool(hit and hit.get("hit"))

    def _drive_rollers(self, conveyor: Dict[str, Any], speed_mm_s: float) -> None:
        """Set every roller's surface speed, in mm/s along the belt."""
        from pxr import Gf, UsdPhysics

        import numpy as np

        stage = self._isaac.omni_usd.get_context().get_stage()
        radius = conveyor["radius_mm"]
        if conveyor.get("mode") == "belt":
            # A flat belt is driven by a SURFACE velocity: the belt does not move, its
            # surface does, and contact is solved as though it were sliding. That is a
            # different thing from the rigid body's own velocity, which is what
            # `ConveyorBeltDemo` sets on its "linear conveyor" - and why that demo has to
            # reverse the sign halfway through, because its belt is genuinely travelling.
            # Set that way here the belt carried itself off up the cell and dropped the
            # part on the floor at y -2774, beyond the end of its own deck.
            #
            # The flat deck is the whole reason for moving off the roller unit: the
            # mailbox stands on rails 7 mm proud of its underside running along its
            # length, and A05's rollers sit on a 68.5 mm pitch with 10.6 mm of air
            # between them, so a rail drops into a gap and wedges. Measured at about one
            # feed in six, and reversing did not clear it - the part re-wedged in the
            # same place. A continuous surface has nowhere for a rail to fall.
            from pxr import PhysxSchema

            for driven in conveyor["rollers"]:
                prim = stage.GetPrimAtPath(driven["path"])
                if not prim or not prim.IsValid():
                    continue
                api = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(prim)
                api.CreateSurfaceVelocityEnabledAttr().Set(abs(speed_mm_s) > 0.0)
                # In WORLD space. The default is the body's own frame, and this belt sits
                # under a prop yawed -90 degrees, so a local +y push drives the part
                # sideways into a guide rail instead of along the deck: measured, it
                # crept backwards at 4 mm/s against the 300 asked for.
                api.CreateSurfaceVelocityLocalSpaceAttr().Set(False)
                # Which way is WORLD, and stated per conveyor rather than assumed.
                #
                # This was a hard-coded world +y, which is right for the infeed - yawed
                # -90 so it runs along y - and wrong for the outfeeds, which run along x.
                # Deriving it from the prop's yaw would be guessing at the asset's own
                # belt axis; the config says it.
                axis = conveyor.get("axis") or (0.0, 1.0, 0.0)
                api.CreateSurfaceVelocityAttr().Set(
                    Gf.Vec3f(*(float(a) * float(speed_mm_s) / 1000.0 for a in axis)))
            return
        # v = omega * r, and a positive speed carries the part towards +y. For a cylinder
        # whose axis is +x the top of the surface moves -y under a positive omega_x, so
        # the sign is inverted.
        #
        # `physics:angularVelocity` is in the WORLD frame, and the same value therefore
        # belongs on every roller. This asset models its rollers in two orientations 180
        # degrees apart - 14 one way, 15 the other - which looks exactly like a reason to
        # convert the spin into each roller's own frame. Doing that is what CREATES a tug
        # of war: alternate rollers then drive in opposite directions in world, 15 against
        # 14, and the part crawls along on the odd roller's worth of net drive. Measured:
        # 292 mm/s uniform, 70 mm/s converted. The orientations are a red herring.
        omega_deg = -math.degrees(speed_mm_s / radius) if radius else 0.0
        for roller in conveyor["rollers"]:
            prim = stage.GetPrimAtPath(roller["path"])
            if prim and prim.IsValid():
                UsdPhysics.RigidBodyAPI(prim).GetAngularVelocityAttr().Set(
                    Gf.Vec3f(omega_deg, 0.0, 0.0))

    # ------------------------------------------------------------------
    # indexing: drive belts until a prim arrives, decided every physics step
    # ------------------------------------------------------------------

    def index(self, action: str, spec: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """Run conveyors until a carried prim arrives at a target, stopping it in the sim.

        A client that polls the pallet over gRPC and then commands a stop lands wherever
        its latency puts it, and that distance scales with how fast the sim runs: tuned at
        0.33 of real time, the same loop stopped 27.7 mm out at 1.0 (2026-10-14). Decided
        here, every physics step, it is the same at any rate.

        action "start" with spec:
          conveyors [names], prim (path), axis (0/1/2), target_mm (world, along axis),
          tol_mm, fast_mm_s, creep_mm_s (the slowest that beats stiction),
          accel_mm_s2 (launch AND braking), still_mm, still_s, max_retries, cut_mm
        action "read": the state. action "stop": stop the belts, drop the index.
        The state: phase (drive / settle / creep / done / failed), x_mm, error_mm, and a
        record of each stop (belt speed and pallet speed at the cut, where it came to rest).
        """
        self._require_booted()
        if self.mock:
            return {"error": "mock"}
        if action == "start":
            spec = dict(spec or {})
            names = [str(n) for n in spec["conveyors"]]
            for n in names:
                if n not in self._conveyors:
                    raise ValueError(f"no conveyor {n!r}")
            st = {
                "conveyors": names, "prim": str(spec["prim"]), "axis": int(spec.get("axis", 0)),
                "target": float(spec["target_mm"]), "tol": float(spec.get("tol_mm", 1.5)),
                "fast": float(spec.get("fast_mm_s", 420.0)),
                "creep": float(spec.get("creep_mm_s", 60.0)),
                "accel": float(spec.get("accel_mm_s2", 800.0)),
                "still_mm": float(spec.get("still_mm", 0.05)),
                "still_s": float(spec.get("still_s", 0.25)),
                "retries": int(spec.get("max_retries", 3)),
                "cut": float(spec.get("cut_mm", 0.75)),
                "phase": "drive", "cmd": 0.0, "t": 0.0, "x": None, "ref": 0.0, "ref_t": 0.0,
                "v": 0.0, "stops": [], "tries": 0, "sign": 0.0,
            }

            def begin() -> Dict[str, Any]:
                st["t0"] = st["now"] = float(self.world.current_time)
                st["x"] = self._index_x(st)
                err = st["target"] - st["x"]
                st["sign"] = 1.0 if err > 0 else -1.0
                if abs(err) <= st["tol"]:
                    st["phase"] = "done"
                self._index_state = st
                if "conveyor_index" not in self._index_cb:
                    self.world.add_physics_callback("conveyor_index", self._index_step)
                    self._index_cb.add("conveyor_index")
                return self._index_report(st)
            return self.run(begin)
        if action == "stop":
            def halt() -> Dict[str, Any]:
                st = self._index_state
                if st is not None:
                    self._index_drive(st, 0.0)
                    if st["phase"] not in ("done", "failed"):
                        st["phase"] = "stopped"
                self._index_state = None
                return self._index_report(st) if st else {"phase": "idle"}
            return self.run(halt)
        st = self._index_state
        return self.run(lambda: self._index_report(st)) if st else {"phase": "idle"}

    def _index_x(self, st: Dict[str, Any]) -> float:
        from pxr import Usd, UsdGeom

        stage = self._isaac.omni_usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(st["prim"])
        m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        return float(m.ExtractTranslation()[st["axis"]]) * 1000.0

    def _index_report(self, st: Dict[str, Any]) -> Dict[str, Any]:
        return {"phase": st["phase"], "x_mm": round(st["x"], 2) if st["x"] is not None else None,
                "error_mm": (round(st["target"] - st["x"], 2) if st["x"] is not None else None),
                "speed_mm_s": round(st["v"], 1), "belt_mm_s": round(st["cmd"], 1),
                "t_s": round(st["t"], 3), "tries": st["tries"], "stops": list(st["stops"])}

    def _index_drive(self, st: Dict[str, Any], speed: float) -> None:
        speed = round(float(speed), 1)
        if speed == st["cmd"]:
            return
        for n in st["conveyors"]:
            c = self._conveyors[n]
            c["speed_mm_s"] = speed
            c["blocked"] = False
            self._drive_rollers(c, speed)
            c["running"] = speed != 0.0
        st["cmd"] = speed

    def _index_step(self, dt: float) -> None:
        # Timed on the world's clock, not the callback's step size: summed after a
        # restage, the step sizes came to 22 s over 2.8 s of sim (2026-10-14).
        st = self._index_state
        if st is None or st["phase"] in ("done", "failed", "stopped"):
            return
        now = float(self.world.current_time)
        dt = now - st["now"]
        if dt <= 0.0:
            return
        x = self._index_x(st)
        st["t"] = now - st["t0"]
        if st["phase"] == "settle":
            # Still = within still_mm of where it was, for still_s of sim. By position:
            # USD is written back once per RENDERED frame, so per physics step the pose
            # reads as moving and stopped by turns.
            if abs(x - st["ref"]) > st["still_mm"]:
                st["ref"], st["ref_t"] = x, now
            st["x"], st["now"] = x, now
            err = st["target"] - x
            if now - st["ref_t"] < st["still_s"]:
                return
            st["v"] = 0.0
            stop = st["stops"][-1]
            stop["rest_err"] = round(err, 2)
            # How far it travelled after the cut, per mm/s it was doing then: learned, and
            # cut that much early from then on. On Isaac Sim 6.1 the pallet does not coast;
            # on 5.0 it went ~1 mm on from a 60 mm/s creep and hunted +-2 mm across the
            # target, never inside it (2026-10-17).
            v_cut = float(stop["v_at_cut"])
            if abs(v_cut) > 1.0:
                on = (float(stop["err_at_cut"]) - err) * (1.0 if v_cut > 0 else -1.0)
                k = min(0.2, max(0.0, on / abs(v_cut)))
                old = self._index_coast_s
                self._index_coast_s = k if old is None else 0.5 * (old + k)
            if abs(err) <= st["tol"]:
                st["phase"] = "done"
            elif st["tries"] >= st["retries"]:
                st["phase"] = "failed"
            else:
                st["tries"] += 1
                st["sign"] = 1.0 if err > 0 else -1.0
                st["phase"] = "creep"
            return
        if x != st["x"]:
            # Only on a fresh pose: between two rendered frames it has not been written
            # back yet, which is neither a stop nor a reason to skip driving the belt - a
            # pallet starting at rest never moves until the belt is driven.
            st["v"] = (x - st["x"]) / dt
            st["x"], st["now"] = x, now
        err = st["target"] - x
        if st["phase"] in ("drive", "creep"):
            sign = 1.0 if err > 0 else -1.0
            # Cut close to the target whatever the window: the pallet does not coast (cut
            # and rest agreed to 0.01 mm over twelve stops, 2026-10-14), so the window is
            # slack for the next stop to use, not somewhere to aim.
            lead = abs(st["v"]) * (self._index_coast_s or 0.0)
            if abs(err) <= max(min(st["tol"] * 0.5, st["cut"]), lead) or sign != st["sign"]:
                # Cut the belt and let the pallet come to rest, recording what it cost.
                st["stops"].append({"err_at_cut": round(err, 2), "v_at_cut": round(st["v"], 1),
                                    "belt_at_cut": st["cmd"], "lead": round(lead, 2)})
                self._index_drive(st, 0.0)
                st["phase"], st["ref"], st["ref_t"] = "settle", x, now
                return
            if st["phase"] == "creep":
                # Each retry at half the last creep's speed, to 15 mm/s: the pallet's pose
                # moves a creep's worth between readings, and on Isaac Sim 5.0 coasts about as
                # far again after the cut - 60 mm/s hunted +-2 mm across a 1.5 mm window.
                self._index_drive(st, sign * max(15.0, st["creep"] / 2.0 ** max(0, st["tries"] - 1)))
                return
            # Launch and brake on the same acceleration: the speed the remaining distance
            # can shed down to creep, the speed the ramp has reached, and the top speed.
            brake = math.sqrt(st["creep"] ** 2 + 2.0 * st["accel"] * max(0.0, abs(err) - st["cut"] - 2.0))
            ramp = st["creep"] + st["accel"] * st["t"]
            self._index_drive(st, sign * max(st["creep"], min(st["fast"], brake, ramp)))
            return

    def conveyor(self, name: str, run: Optional[bool] = None,
                 speed_mm_s: Optional[float] = None) -> Dict[str, Any]:
        """Start, stop or report a conveyor."""
        if name not in self._conveyors:
            raise ValueError(f"no conveyor {name!r}; have {sorted(self._conveyors)}")
        conveyor = self._conveyors[name]
        if speed_mm_s is not None:
            conveyor["speed_mm_s"] = float(speed_mm_s)
        if run is not None:
            def apply() -> None:
                if run:
                    conveyor["blocked"] = False
                    self._drive_rollers(conveyor, conveyor["speed_mm_s"])
                else:
                    self._drive_rollers(conveyor, 0.0)
                conveyor["running"] = bool(run)
            self.run(apply)
        return {
            "running": conveyor["running"],
            "blocked": self.run(lambda: self._beam_blocked(conveyor)),
            "stopped_by_beam": conveyor["blocked"],
            "speed_mm_s": conveyor["speed_mm_s"],
            "rollers": len(conveyor["rollers"]),
            "radius_mm": round(conveyor["radius_mm"], 2),
            "beam_y_mm": conveyor["beam_y_mm"],
        }

    def _make_static(self, prim_path: str) -> str:
        """Stop a referenced asset moving, and say what was holding it up.

        `fixed` used to mean only "report this to the planner as scenery". It did not
        reach the simulator, so an asset that shipped its own rigid bodies was scenery
        to the planner and cargo to physics - the two disagreeing quietly, which is the
        worst way for them to disagree.

        The body is made KINEMATIC rather than disabled or removed. All three stop it
        falling, but they are not equivalent later: a conveyor is scenery that has to be
        driven, and the way isaac drives one - `ConveyorBeltDemo` and the
        `isaacsim.asset.gen.conveyor` node alike - is a surface velocity on a kinematic
        rigid body. Disabling the body would have made T1 pass and left T2 with nothing
        to drive. Removing the API would strip the mass and collision settings authored
        alongside it.
        """
        from pxr import Usd, UsdPhysics

        stage = self._isaac.omni_usd.get_context().get_stage()
        root = stage.GetPrimAtPath(prim_path)
        if not root or not root.IsValid():
            raise ValueError(f"{prim_path}: not in the stage")
        frozen = []
        for prim in Usd.PrimRange(root):
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            body = UsdPhysics.RigidBodyAPI(prim)
            body.CreateKinematicEnabledAttr().Set(True)
            frozen.append(prim.GetPath().pathString)
        self._static_notes[prim_path] = frozen
        LOGGER.info("prop %s: fixed, made %d rigid bodies kinematic: %s",
                    prim_path, len(frozen), frozen[:8])
        return f"{len(frozen)} rigid bodies made kinematic"

    def _add_usd_colliders(self, prim_path: str, approximation: str,
                           scopes: List[str]) -> None:
        """Give an asset's visual meshes collision, since this one ships without any.

        `approximation` is a UsdPhysics token - convexHull, boundingCube,
        meshSimplification, none. A convex hull is right for handling a shell like this:
        the mailbox's tunnel is concave and a hull fills the opening, which costs nothing
        because nothing in the cell reaches inside it.
        """
        from pxr import Usd, UsdGeom, UsdPhysics

        stage = self._isaac.omni_usd.get_context().get_stage()
        root = stage.GetPrimAtPath(prim_path)
        if not root or not root.IsValid():
            raise ValueError(f"{prim_path}: not in the stage, cannot add colliders")
        wanted = set(scopes)
        made = 0
        for prim in Usd.PrimRange(root):
            if not prim.IsA(UsdGeom.Gprim):
                continue
            if not any(part in wanted for part in prim.GetPath().pathString.split("/")):
                continue
            UsdPhysics.CollisionAPI.Apply(prim)
            mesh = UsdPhysics.MeshCollisionAPI.Apply(prim)
            mesh.CreateApproximationAttr(approximation)
            made += 1
        if not made:
            raise ValueError(
                f"{prim_path}: collision_approximation was asked for but no geometry was "
                f"found under {sorted(wanted)} - nothing would collide")
        LOGGER.info("prop %s: %d colliders added as %s under %s",
                    prim_path, made, approximation, sorted(wanted))

    def _set_usd_mass(self, prim_path: str, mass_kg: float) -> None:
        """Put a mass on whichever prims under here are rigid bodies.

        A referenced asset states its own mass - this mailbox says 2.5 kg, which is right
        for the real thing and wrong for one scaled to a third of its length. Scaling a
        prim does not rescale an explicitly authored mass, so without this a 182 mm part
        weighs what a 495 mm one does.
        """
        from pxr import Usd, UsdPhysics

        stage = self._isaac.omni_usd.get_context().get_stage()
        root = stage.GetPrimAtPath(prim_path)
        if not root or not root.IsValid():
            raise ValueError(f"{prim_path}: not in the stage, cannot set mass")
        bodies = [prim for prim in Usd.PrimRange(root)
                  if prim.HasAPI(UsdPhysics.RigidBodyAPI)]
        if not bodies:
            bodies = [root]
        # Split by how big each body actually is, not evenly. This mailbox is a shell and
        # a hinged door, and an even split makes the door weigh as much as everything else
        # - a door that heavy swinging on its hinge while the arm carries the part is a
        # moving load on the wrist, which is the failure this cell spent a long time
        # learning to recognise.
        volumes = []
        for prim in bodies:
            box = self._bbox_cache().ComputeWorldBound(prim).ComputeAlignedRange()
            if box.IsEmpty():
                volumes.append(0.0)
                continue
            lo, hi = box.GetMin(), box.GetMax()
            volumes.append(max(
                (hi[0] - lo[0]) * (hi[1] - lo[1]) * (hi[2] - lo[2]), 0.0))
        total = sum(volumes)
        if total <= 0.0:
            volumes = [1.0] * len(bodies)
            total = float(len(bodies))
        for prim, volume in zip(bodies, volumes):
            api = UsdPhysics.MassAPI.Apply(prim)
            api.CreateMassAttr(mass_kg * volume / total)
        LOGGER.info("prop %s: mass %.3f kg over %d rigid bodies, by volume %s",
                    prim_path, mass_kg, len(bodies),
                    [round(v / total, 3) for v in volumes])

    def _bbox_cache(self):
        """A bounding-box cache over the default and render purposes."""
        from pxr import Usd, UsdGeom

        # `guide` as well as the visible purposes. Collision geometry is conventionally
        # authored as guide so it does not render, and a cache that omits that purpose
        # reports an empty bound for it - which reads exactly like an asset with no
        # colliders at all. Measured: this mailbox's Body/Collision came back with NO
        # EXTENT until guide was included, and its colliders were there the whole time.
        return UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.guide])

    def _measured_bounds(self, prop_name: str) -> Optional[Dict[str, List[float]]]:
        """A usd prop's extent about its own origin, measured from the stage.

        So `bounds_m` is an override rather than a requirement. An asset states its size in
        its geometry, and the module can read that now - which is better than a number in
        the config that has to be kept in step by hand, and much better than the previous
        behaviour of quietly leaving an asset out of the obstacle list when the number was
        missing.
        """
        path = f"/World/{_prim_name(prop_name)}"
        try:
            got = self.prim_bounds([path]).get(path)
        except Exception:  # noqa: BLE001
            return None
        if not got:
            return None
        here = self.prop_poses([prop_name]).get(prop_name, {}).get("position_mm")
        if here is None:
            return None
        origin = [float(v) / 1000.0 for v in here]
        return {
            "min": [got["min"][i] - origin[i] for i in range(3)],
            "max": [got["max"][i] - origin[i] for i in range(3)],
        }

    def raycast(self, origin_mm: List[float], direction: List[float],
                distance_mm: float = 200.0) -> Dict[str, Any]:
        """What PhysX hits along a ray. The question a suction cup actually asks.

        Bounding boxes say where geometry IS; a scene query says what the simulator will
        REPORT, and those are different things - a collider can be composed into the stage
        and still not be in the physics scene. The gripper finds its payload by raycast, so
        when it reports nothing on a face that measures solid, this is the only way to tell
        which of the two is wrong.
        """
        self._require_booted()
        return self.run(lambda: self._raycast_raw(origin_mm, direction, distance_mm))

    def _raycast_raw(self, origin_mm: List[float], direction: List[float],
                     distance_mm: float = 200.0) -> Dict[str, Any]:
        """The cast itself, ON the sim thread.

        Split out from `raycast` because the break beam is checked from inside a physics
        callback, which already runs on the sim thread - going through `run` from there
        would wait on the thread it is already on.
        """
        from omni.physx import get_physx_scene_query_interface

        origin = [float(v) / 1000.0 for v in origin_mm]
        length = float(max(v * v for v in direction)) ** 0.5
        if length <= 0:
            raise ValueError("raycast needs a non-zero direction")
        unit = [float(v) / length for v in direction]
        hit = get_physx_scene_query_interface().raycast_closest(
            origin, unit, float(distance_mm) / 1000.0)
        if not hit or not hit.get("hit"):
            return {"hit": False, "from_mm": [round(v * 1000.0, 1) for v in origin],
                    "along": [round(v, 3) for v in unit],
                    "looked_mm": float(distance_mm)}
        return {
            "hit": True,
            "body": str(hit.get("rigidBody", "")),
            "collider": str(hit.get("collision", "")),
            "distance_mm": round(float(hit.get("distance", 0.0)) * 1000.0, 2),
            "at_mm": [round(float(v) * 1000.0, 1)
                      for v in (hit.get("position") or [0, 0, 0])],
        }

    def prim_attrs(self, paths: List[str]) -> Dict[str, Any]:
        """Every authored attribute on a prim, plus its schemas and instancing state.

        A read-only window on the composed stage, for the questions that cannot be
        answered from the config or from a bounding box. Written because the belt was
        measured as not moving at all with a conveyor reporting `running: True`, and the
        difference between "the surface velocity is not authored", "it is authored on an
        instance proxy where it does nothing" and "it is authored and PhysX is ignoring
        it" is three different fixes and not visible from outside.

        Authored values only: an unauthored attribute's fallback says what the schema
        would do, not what this stage says, and reporting the two the same way is how a
        default gets mistaken for a setting.
        """
        self._require_booted()
        if self.mock:
            return {p: {"error": "mock"} for p in paths}

        def _read() -> Dict[str, Any]:
            stage = self._isaac.omni_usd.get_context().get_stage()
            out: Dict[str, Any] = {}
            for path in paths:
                prim = stage.GetPrimAtPath(path)
                if not prim or not prim.IsValid():
                    out[path] = {"error": "not in the stage"}
                    continue
                attrs: Dict[str, Any] = {}
                for attr in prim.GetAttributes():
                    if not attr.HasAuthoredValue():
                        continue
                    try:
                        value = attr.Get()
                    except Exception:  # noqa: BLE001
                        attrs[attr.GetName()] = "<unreadable>"
                        continue
                    attrs[attr.GetName()] = (
                        value if isinstance(value, (bool, int, float, str))
                        else str(value))
                out[path] = {
                    "type": str(prim.GetTypeName()),
                    "schemas": [str(x) for x in prim.GetAppliedSchemas()],
                    "instanceable": bool(prim.IsInstanceable()),
                    "instance": bool(prim.IsInstance()),
                    "instance_proxy": bool(prim.IsInstanceProxy()),
                    "in_prototype": bool(prim.IsInPrototype()),
                    "children": [c.GetName() for c in prim.GetChildren()][:12],
                    "attrs": attrs,
                }
            return out

        return self.run(_read)

    def prim_bounds(self, paths: List[str]) -> Dict[str, Any]:
        """Each prim's world-aligned extent, in metres. TRUSTWORTHY ONLY FOR USD ASSETS.

        Measured against a cube prop whose config states it exactly: a belt of size 1.0
        scaled [0.18, 1.3415, 0.076] came back as 32 x 1800 x 6 mm rather than
        180 x 1341 x 76 - the scale applied twice on every axis. Isaac's cuboid helper
        bakes `scale` into the geometry AND sets an xform scale, so the composed bound
        double counts. A referenced asset scaled with set_local_scale measures correctly,
        which is what this exists for.

        So do not use this to ask how tall the belt is; the config says that, and
        `prop_obstacles` does the arithmetic. The part rests with its underside exactly on
        the belt top the config computes, which is the evidence that the config is right
        and this is the thing that is wrong.

        What a defect decal's FACE is derived from. The cube part carried its marks as
        children the fragment could be read for, so the cell worked out which face each
        one sat on from the config alone; a referenced asset carries them as geometry, and
        the only honest source for where they are is the stage. Same derivation either
        way - the axis a mark is furthest out along names its face - just read from the
        composed scene instead of from json.
        """
        self._require_booted()
        from pxr import Usd, UsdGeom

        def _read():
            stage = self._isaac.omni_usd.get_context().get_stage()
            cache = self._bbox_cache()
            out: Dict[str, Any] = {}
            for path in paths:
                prim = stage.GetPrimAtPath(path)
                if not prim or not prim.IsValid():
                    out[path] = None
                    continue
                box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
                if box.IsEmpty():
                    out[path] = None
                    continue
                lo, hi = box.GetMin(), box.GetMax()
                out[path] = {
                    "min": [float(v) for v in lo],
                    "max": [float(v) for v in hi],
                    "centre": [float((a + b) / 2.0) for a, b in zip(lo, hi)],
                }
            return out

        return self.run(_read)

    def free_kinematic_near(self, prim_path: str, radius_mm: float) -> List[str]:
        """Make dynamic every prop `set_kinematic` holds that lies within `radius_mm` of
        `prim_path` (a gripper's flange). Returns the props freed."""
        self._require_booted()
        held = sorted(getattr(self, "_runtime_kinematic", set()))
        if self.mock or not held:
            return []
        import numpy as np
        from pxr import Usd, UsdPhysics

        def _free() -> List[str]:
            stage = self._isaac.omni_usd.get_context().get_stage()
            at = np.array(self._isaac.SingleXFormPrim(prim_path).get_world_pose()[0])
            freed = []
            for name in held:
                root = stage.GetPrimAtPath(f"/World/{_prim_name(name)}")
                if not root or not root.IsValid():
                    continue
                where = np.array(self._isaac.SingleXFormPrim(
                    root.GetPath().pathString).get_world_pose()[0])
                if np.linalg.norm(where - at) * 1000.0 > radius_mm:
                    continue
                for prim in Usd.PrimRange(root):
                    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                        UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(False)
                freed.append(name)
            for name in freed:
                self._runtime_kinematic.discard(name)
            return freed

        return self.run(_free)

    def set_kinematic(self, name: str, on: bool,
                      position_mm: Optional[List[float]] = None,
                      orientation_wxyz: Optional[List[float]] = None) -> Dict[str, Any]:
        """Freeze a dynamic prop where it is told to be, or let it go again.

        For a part that waits at a station before an arm takes it: resting in a nest it is
        thrown by every scene re-read and settles wherever it lands (the flag, 15 mm and
        10 mm up in four seconds, 2026-10-13). Kinematic, it sits exactly where it is put;
        it is made dynamic again as the gripper closes on it.

        Turning it ON with a pose writes the body's TRANSFORM, which is what a kinematic
        body follows (its rigid view's pose is overwritten every step).
        """
        self._require_booted()
        if self.mock:
            return {"kinematic": on}
        import numpy as np
        from pxr import Usd, UsdPhysics

        prim_path = f"/World/{_prim_name(name)}"

        def _set() -> Dict[str, Any]:
            stage = self._isaac.omni_usd.get_context().get_stage()
            root = stage.GetPrimAtPath(prim_path)
            if not root or not root.IsValid():
                raise ValueError(f"{prim_path}: not in the stage")
            bodies = [prim for prim in Usd.PrimRange(root)
                      if prim.HasAPI(UsdPhysics.RigidBodyAPI)]
            for prim in bodies:
                UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(bool(on))
            if on and position_mm is not None and bodies:
                where = np.array([float(v) / 1000.0 for v in position_mm])
                quat = (np.array([float(v) for v in orientation_wxyz])
                        if orientation_wxyz is not None else None)
                self._isaac.SingleXFormPrim(bodies[0].GetPath().pathString).set_world_pose(
                    position=where, orientation=quat)
            if not on:
                for prim in bodies:
                    try:
                        h = self._isaac.SingleRigidPrim(prim.GetPath().pathString)
                        h.set_linear_velocity(np.zeros(3))
                        h.set_angular_velocity(np.zeros(3))
                    except Exception:  # noqa: BLE001
                        pass
            if not hasattr(self, "_runtime_kinematic"):
                self._runtime_kinematic = set()
            (self._runtime_kinematic.add if on else self._runtime_kinematic.discard)(name)
            return {"prop": name, "kinematic": bool(on), "bodies": len(bodies)}

        return self.run(_set)

    def profile(self, steps: int = 60, hide: Optional[List[str]] = None,
                render_every: Optional[List[int]] = None) -> Dict[str, Any]:
        """Where a step's wall time goes, measured on the sim thread in one pass.

        Times `steps` steps in each configuration and puts everything back:
          physics    - world.step(render=False): PhysX plus every physics callback
          render     - world.step(render=True): the above plus kit's frame (viewport,
                       livestream, every live camera render product)
          no_viewport - render with the kit viewport's updates off (what the livestream,
                       which streams that viewport, costs)
          viewport_640x360 - render with the viewport at 640x360
          hidden     - render with each prim in `hide` made invisible (render cost only;
                       PhysX still has its colliders)
          no_viewport_hidden - both
          every_<k>  - render one step in k (what rendering_dt = k * physics_dt does)
          no_cameras - render with every camera render product's updates off
          no_viewport_no_cameras - both off: kit's frame with nothing to draw
        Plus which camera render products are live, and the loop's cumulative split.
        """
        self._require_booted()
        if self.mock:
            return {"error": "mock"}
        n = max(1, int(steps))
        hide = [str(p) for p in (hide or [])]

        def timed(render: bool, every: int = 1) -> float:
            t = time.perf_counter()
            for i in range(n):
                self.world.step(render=render and (i % every == every - 1))
            return round((time.perf_counter() - t) / n * 1000.0, 2)

        def body() -> Dict[str, Any]:
            from pxr import UsdGeom

            out: Dict[str, Any] = {"steps": n,
                                   "physics_dt": float(self.world.get_physics_dt()),
                                   "rendering_dt": float(self.world.get_rendering_dt())}
            ms: Dict[str, float] = {}
            ms["render"] = timed(True)
            ms["physics"] = timed(False)
            vp = None
            try:
                from omni.kit.viewport.utility import get_active_viewport
                vp = get_active_viewport()
                vp_was = bool(vp.updates_enabled) if vp is not None else True
            except Exception as e:
                out["viewport_error"] = str(e)
            stage = self._isaac.omni_usd.get_context().get_stage()
            prims = [stage.GetPrimAtPath(p) for p in hide]
            prims = [p for p in prims if p and p.IsValid()]
            out["hidden"] = [str(p.GetPath()) for p in prims]
            was = [UsdGeom.Imageable(p).ComputeVisibility() for p in prims]
            try:
                if vp is not None:
                    vp.updates_enabled = False
                    ms["no_viewport"] = timed(True)
                    vp.updates_enabled = vp_was
                    try:
                        res = tuple(vp.resolution)
                        out["viewport_resolution"] = list(res)
                        vp.resolution = (640, 360)
                        ms["viewport_640x360"] = timed(True)
                        vp.resolution = res
                    except Exception as e:
                        out["viewport_resolution_error"] = str(e)
                if prims:
                    for p in prims:
                        UsdGeom.Imageable(p).MakeInvisible()
                    ms["hidden"] = timed(True)
                    if vp is not None:
                        vp.updates_enabled = False
                        ms["no_viewport_hidden"] = timed(True)
            finally:
                if vp is not None:
                    vp.updates_enabled = vp_was
                for p, v in zip(prims, was):
                    if v != UsdGeom.Tokens.invisible:
                        UsdGeom.Imageable(p).MakeVisible()
            for k in (render_every or []):
                ms[f"every_{int(k)}"] = timed(True, max(1, int(k)))
            texs = [h._texture() for h in self._camera_handles]
            texs = [t for t in texs if t is not None]
            was_on = []
            for t in texs:
                try:
                    was_on.append(bool(t.get_updates_enabled()))
                except Exception:
                    was_on.append(True)
            try:
                for t in texs:
                    t.set_updates_enabled(False)
                ms["no_cameras"] = timed(True)
                if vp is not None:
                    vp.updates_enabled = False
                    ms["no_viewport_no_cameras"] = timed(True)
                    vp.updates_enabled = vp_was
                for k in (render_every or []):
                    ms[f"no_cameras_every_{int(k)}"] = timed(True, max(1, int(k)))
            finally:
                for t, on in zip(texs, was_on):
                    t.set_updates_enabled(on)
            out["ms_per_step"] = ms
            out["rtf"] = {k: round(out["physics_dt"] * 1000.0 / v, 3) for k, v in ms.items() if v}
            live = []
            for h in self._camera_handles:
                tex = h._texture()
                try:
                    on = bool(tex.get_updates_enabled()) if tex is not None else True
                except Exception:
                    on = h._live
                if on:
                    live.append(str(getattr(h._cam, "prim_path", "?")))
            out["live_cameras"] = live
            out["cameras"] = len(self._camera_handles)
            return out

        res = self.run(body, timeout=600.0)
        res["loop_s"] = dict(self._loop_s)
        return res

    def render_probe(self, cameras: Optional[List[str]] = None, frames: int = 4) -> Dict[str, Any]:
        """How many rendered frames a camera's read lags its render product's updates.

        For each camera (prim path), a red cube is put 0.3 m down its view and toggled, and
        the frame read after each step is compared with the frame without it: mean absolute
        difference, 0 = the old frame. Cases: updates left on; updates off then on for
        `frames` steps; and on for exactly one step then off BEFORE the read. Plus ms per
        step with one camera toggled on and off every frame. Everything put back.
        """
        self._require_booted()
        if self.mock:
            return {"error": "mock"}

        def body() -> Dict[str, Any]:
            import numpy as np
            from pxr import Gf, UsdGeom

            stage = self._isaac.omni_usd.get_context().get_stage()
            out: Dict[str, Any] = {}
            for h in self._camera_handles:
                path = str(getattr(h._cam, "prim_path", "?"))
                if cameras and path not in cameras:
                    continue
                tex = h._texture()
                if tex is None:
                    continue
                cam = h._cam
                was_on = bool(tex.get_updates_enabled())
                m = UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(0)
                at = m.ExtractTranslation()
                fwd = -Gf.Vec3d(m.GetRow3(2)).GetNormalized()
                cube = UsdGeom.Cube.Define(stage, "/World/_render_probe")
                cube.CreateSizeAttr(0.15)
                cube.CreateDisplayColorAttr([Gf.Vec3f(1.0, 0.0, 0.0)])
                UsdGeom.XformCommonAPI(cube).SetTranslate(at + fwd * 0.3)
                img = UsdGeom.Imageable(cube.GetPrim())

                def grab():
                    f = cam.get_rgba()
                    return None if f is None or f.size == 0 else f[:, :, :3].astype("float32")

                def d(a, b):
                    return None if a is None or b is None else round(float(np.abs(a - b).mean()), 2)

                def step():
                    self.world.step(render=True)

                r: Dict[str, Any] = {}
                try:
                    img.MakeInvisible()
                    tex.set_updates_enabled(True)
                    for _ in range(4):
                        step()
                    clear = grab()
                    img.MakeVisible()
                    seq = []
                    for _ in range(frames):
                        step()
                        seq.append(d(grab(), clear))
                    r["on_show_cube"] = seq
                    shown = grab()
                    img.MakeInvisible()
                    tex.set_updates_enabled(False)
                    for _ in range(3):
                        step()
                    r["off_3_steps_vs_cube"] = d(grab(), shown)
                    tex.set_updates_enabled(True)
                    seq = []
                    for _ in range(frames):
                        step()
                        seq.append(d(grab(), shown))
                    r["off_then_on_vs_cube"] = seq
                    img.MakeVisible()
                    tex.set_updates_enabled(False)
                    for _ in range(3):
                        step()
                    tex.set_updates_enabled(True)
                    step()
                    tex.set_updates_enabled(False)
                    one = grab()
                    r["one_step_then_off_vs_clear"] = d(one, clear)
                    step()
                    r["one_more_step_off_vs_clear"] = d(grab(), clear)
                    n = 10
                    t = time.perf_counter()
                    for _ in range(n):
                        tex.set_updates_enabled(True)
                        step()
                        tex.set_updates_enabled(False)
                    r["ms_toggled_every_step"] = round((time.perf_counter() - t) / n * 1000, 2)
                    tex.set_updates_enabled(True)
                    t = time.perf_counter()
                    for _ in range(n):
                        step()
                    r["ms_left_on"] = round((time.perf_counter() - t) / n * 1000, 2)
                    tex.set_updates_enabled(False)
                    t = time.perf_counter()
                    for _ in range(n):
                        step()
                    r["ms_off"] = round((time.perf_counter() - t) / n * 1000, 2)
                except Exception as e:                                   # noqa: BLE001
                    r["error"] = repr(e)
                finally:
                    stage.RemovePrim("/World/_render_probe")
                    tex.set_updates_enabled(was_on)
                out[path] = r
            return out

        return self.run(body, timeout=600.0)

    def toggle_probe(self, camera: str, toggles: int = 20, every: int = 5) -> Dict[str, Any]:
        """GPU memory against render-product toggles, and USD poses without rendering.

        1. memory (nvidia-smi MiB) every `every` toggles of `camera`'s updates on for one
           rendered step and off, then after 60 steps with it off (is it given back?).
        2. A falling rigid cube: its USD pose against PhysX's after physics-only steps, after
           physx update_transformations, and after one rendered step.
        """
        self._require_booted()
        if self.mock:
            return {"error": "mock"}
        import subprocess

        def mib() -> int:
            try:
                return int(subprocess.run(["nvidia-smi", "--query-gpu=memory.used",
                                           "--format=csv,noheader,nounits"],
                                          capture_output=True, text=True).stdout.split()[0])
            except Exception:                                            # noqa: BLE001
                return -1

        def body() -> Dict[str, Any]:
            import carb
            from pxr import Gf, UsdGeom, UsdPhysics

            out: Dict[str, Any] = {}
            st = carb.settings.get_settings()
            out["settings"] = {k: st.get(k) for k in (
                "/physics/updateToUsd", "/physics/updateVelocitiesToUsd",
                "/physics/fabricEnabled", "/physics/suppressReadback",
                "/app/asyncRendering", "/app/runLoops/main/rateLimitEnabled")}
            try:
                import omni.kit.app
                em = omni.kit.app.get_app().get_extension_manager()
                out["fabric_ext"] = bool(em.is_extension_enabled("omni.physx.fabric"))
            except Exception as e:                                       # noqa: BLE001
                out["fabric_ext"] = repr(e)
            h = next((h for h in self._camera_handles
                      if str(getattr(h._cam, "prim_path", "")) == camera), None)
            mem = {"start": mib()}
            if h is not None and h._texture() is not None:
                tex = h._texture()
                was = bool(tex.get_updates_enabled())
                seq = []
                try:
                    for i in range(int(toggles)):
                        tex.set_updates_enabled(True)
                        self.world.step(render=True)
                        tex.set_updates_enabled(False)
                        if (i + 1) % int(every) == 0:
                            seq.append(mib())
                    mem["per_toggles"] = seq
                    for _ in range(60):
                        self.world.step(render=True)
                    mem["after_60_off"] = mib()
                    tex.set_updates_enabled(True)
                    for _ in range(int(toggles)):
                        self.world.step(render=True)
                    mem["after_left_on"] = mib()
                finally:
                    tex.set_updates_enabled(was)
            out["gpu_mib"] = mem
            stage = self._isaac.omni_usd.get_context().get_stage()
            path = "/World/_toggle_probe_body"
            try:
                cube = UsdGeom.Cube.Define(stage, path)
                cube.CreateSizeAttr(0.05)
                UsdGeom.XformCommonAPI(cube).SetTranslate(Gf.Vec3d(-3.0, -3.0, 3.0))
                UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
                UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
                from omni.physx import get_physx_interface
                px = get_physx_interface()

                def usd_z():
                    return round(float(UsdGeom.Xformable(cube.GetPrim())
                                       .ComputeLocalToWorldTransform(0).ExtractTranslation()[2]), 4)

                def px_z():
                    try:
                        return round(float(px.get_rigidbody_transformation(path)["position"][2]), 4)
                    except Exception as e:                               # noqa: BLE001
                        return repr(e)[:80]
                r = {}
                for _ in range(3):
                    self.world.step(render=True)
                r["after_3_rendered"] = (usd_z(), px_z())
                for _ in range(15):
                    self.world.step(render=False)
                r["after_15_physics_only"] = (usd_z(), px_z())
                t = time.perf_counter()
                try:
                    px.update_transformations(False, True, True, False)
                    r["update_transformations_ms"] = round((time.perf_counter() - t) * 1000, 2)
                except Exception as e:                                   # noqa: BLE001
                    r["update_transformations_error"] = repr(e)[:120]
                r["after_update_transformations"] = (usd_z(), px_z())
                for _ in range(15):
                    self.world.step(render=False)
                r["after_15_more_physics_only"] = (usd_z(), px_z())
                self.world.step(render=True)
                r["after_1_rendered"] = (usd_z(), px_z())
                out["usd_vs_physx_z_m"] = r
            except Exception as e:                                       # noqa: BLE001
                out["pose_error"] = repr(e)[:200]
            finally:
                stage.RemovePrim(path)
            out["gpu_mib"]["end"] = mib()
            return out

        return self.run(body, timeout=600.0)

    def leak_probe(self, camera: str, mechanisms: Optional[List[str]] = None,
                   cycles: int = 6, settings: Optional[Dict[str, Any]] = None,
                   hide: Optional[List[str]] = None, keepalive: int = 0) -> Dict[str, Any]:
        """GPU memory per off-on cycle of `camera`, by the mechanism that turns it off.

          updates  - its hydra texture's updates off and on (what set_rendering does)
          paused   - the same, plus the Camera's own acquisition callback paused
          annotators - updates left on; the annotators detached and re-attached
          destroy  - the render product destroyed, and the Camera re-initialized
        Each cycle: on, 4 rendered steps, a read (rgba, depth if attached), off, 4 steps;
        nvidia-smi MiB after each. Plus ms per step while off, against left on.
        """
        self._require_booted()
        if self.mock:
            return {"error": "mock"}
        import subprocess

        def mib() -> int:
            try:
                return int(subprocess.run(["nvidia-smi", "--query-gpu=memory.used",
                                           "--format=csv,noheader,nounits"],
                                          capture_output=True, text=True).stdout.split()[0])
            except Exception:                                            # noqa: BLE001
                return -1

        h = next((h for h in self._camera_handles
                  if str(getattr(h._cam, "prim_path", "")) == camera), None)
        if h is None and camera != "new":
            return {"error": f"no camera at {camera}"}
        cam = h._cam if h is not None else None

        def body() -> Dict[str, Any]:
            nonlocal h, cam, camera
            if cam is None:
                # "new": a plain Camera made here, none of the cell's camera setup
                import types
                import numpy as np
                camera = "/World/_leak_cam"
                cam = self._isaac.Camera(prim_path=camera, resolution=(1280, 720),
                                         position=np.array([0.0, -3.0, 2.0]))
                cam.initialize()
                for _ in range(10):
                    self.world.step(render=True)
                h = types.SimpleNamespace(
                    _cam=cam, _texture=lambda: cam._render_product.hydra_texture)
            import carb
            from pxr import UsdGeom as _G

            depth = "distance_to_image_plane" in getattr(cam, "_custom_annotators", {})
            out: Dict[str, Any] = {"depth": depth}
            cs = carb.settings.get_settings()
            keys = ["/rtx-transient/resourcemanager/enableTextureStreaming",
                    "/rtx-transient/resourcemanager/texturestreaming/memoryBudget",
                    "/rtx-transient/resourcemanager/texturestreaming/streamingBudgetMB",
                    "/rtx-transient/resourcemanager/texturestreaming/evictionFrameLatency",
                    "/rtx/resourcemanager/texturestreaming/gpuBudgetPriority",
                    "/rtx-transient/hydra/geometrystreaming/streamingBudgetMB"]
            out["settings_before"] = {k: cs.get(k) for k in keys}
            for k, v in (settings or {}).items():
                cs.set(k, v)
            out["settings_set"] = {k: cs.get(k) for k in (settings or {})}
            st = self._isaac.omni_usd.get_context().get_stage()
            hidden = []
            for pth in (hide or []):
                pr = st.GetPrimAtPath(pth)
                if pr and pr.IsValid():
                    _G.Imageable(pr).MakeInvisible()
                    hidden.append(pth)
            out["hidden"] = hidden
            if keepalive and not st.GetPrimAtPath("/World/_keepalive_cam"):
                # a tiny render product left on, so the renderer is never idle
                import numpy as np
                t0 = time.perf_counter()
                for _ in range(30):
                    self.world.step(render=True)
                out["ms_before_keepalive"] = round((time.perf_counter() - t0) / 30 * 1000, 2)
                ka = self._isaac.Camera(prim_path="/World/_keepalive_cam",
                                        resolution=(int(keepalive), int(keepalive)),
                                        position=np.array([0.0, 0.0, -50.0]))
                ka.initialize(attach_rgb_annotator=False)
                for _ in range(10):
                    self.world.step(render=True)
                t0 = time.perf_counter()
                for _ in range(30):
                    self.world.step(render=True)
                out["ms_with_keepalive"] = round((time.perf_counter() - t0) / 30 * 1000, 2)
                out["mib_keepalive_added"] = mib()

            def steps(n=4):
                t = time.perf_counter()
                for _ in range(n):
                    self.world.step(render=True)
                return (time.perf_counter() - t) / n * 1000.0

            def read():
                f = cam.get_rgba()
                ok = f is not None and getattr(f, "size", 0) > 0
                if depth:
                    d = cam.get_depth()
                    ok = ok and d is not None and d.size > 0
                return ok

            def tex():
                return h._texture()

            from pxr import Gf, UsdGeom
            stage = self._isaac.omni_usd.get_context().get_stage()
            park = "/World/_park_cam"
            if not stage.GetPrimAtPath(park):
                pc = UsdGeom.Camera.Define(stage, park)
                UsdGeom.XformCommonAPI(pc).SetTranslate(Gf.Vec3d(0.0, 0.0, -500.0))
                pc.CreateClippingRangeAttr(Gf.Vec2f(0.01, 0.02))
            cprim = UsdGeom.Camera(stage.GetPrimAtPath(camera))
            clip0 = cprim.GetClippingRangeAttr().Get()

            def repoint(path):
                from isaacsim.core.utils.render_product import set_camera_prim_path
                set_camera_prim_path(cam._render_product_path, path)

            def on(m):
                if m == "clip":
                    cprim.GetClippingRangeAttr().Set(clip0)
                elif m == "repoint":
                    repoint(camera)
                elif m in ("updates", "paused"):
                    tex().set_updates_enabled(True)
                    if m == "paused":
                        cam.resume()
                elif m == "annotators":
                    cam.attach_annotator(annotator_name="rgb")
                    if depth:
                        cam.add_distance_to_image_plane_to_frame()
                elif m == "destroy":
                    cam.initialize(attach_rgb_annotator=True)
                    if depth:
                        cam.add_distance_to_image_plane_to_frame()

            def off(m):
                if m == "clip":
                    cprim.GetClippingRangeAttr().Set(Gf.Vec2f(0.001, 0.002))
                elif m == "repoint":
                    repoint(park)
                elif m in ("updates", "paused"):
                    if m == "paused":
                        cam.pause()
                    tex().set_updates_enabled(False)
                elif m == "annotators":
                    for name in list(cam._custom_annotators.keys()):
                        cam.detach_annotator(name)
                elif m == "destroy":
                    cam.destroy()

            was = bool(tex().get_updates_enabled()) if tex() is not None else True
            for m in (mechanisms or ["updates", "paused", "annotators", "destroy"]):
                r: Dict[str, Any] = {}
                try:
                    if m in ("annotators", "clip", "repoint") and tex() is not None:
                        tex().set_updates_enabled(True)
                    if m in ("clip", "repoint"):
                        steps(6)
                        ref = cam.get_rgba()
                        ref = None if ref is None else ref[:, :, :3].astype("float32")
                    on(m)
                    steps(6)
                    r["read_ok_first"] = read()
                    r["ms_on"] = round(steps(20), 2)
                    off(m)
                    steps(4)
                    r["ms_off"] = round(steps(20), 2)
                    seq, oks = [mib()], []
                    for _ in range(int(cycles)):
                        on(m)
                        steps(4)
                        oks.append(read())
                        off(m)
                        steps(4)
                        seq.append(mib())
                    r["mib"] = seq
                    r["mib_per_cycle"] = round((seq[-1] - seq[0]) / max(1, int(cycles)), 1)
                    r["reads_ok"] = all(oks)
                    on(m)
                    steps(6)
                    r["read_ok_after"] = read()
                    if m in ("clip", "repoint") and ref is not None:
                        import numpy as np
                        now = cam.get_rgba()[:, :, :3].astype("float32")
                        r["frame_diff_vs_before"] = round(float(np.abs(now - ref).mean()), 3)
                        off(m)
                        steps(6)
                        dark = cam.get_rgba()[:, :, :3].astype("float32")
                        r["off_frame_mean"] = round(float(dark.mean()), 2)
                        on(m)
                        steps(6)
                    r["resolution"] = list(cam.get_resolution())
                except Exception as e:                                   # noqa: BLE001
                    r["error"] = repr(e)[:300]
                    try:
                        on(m)
                    except Exception:                                    # noqa: BLE001
                        pass
                out[m] = r
            if tex() is not None:
                tex().set_updates_enabled(was)
            for pth in hidden:
                _G.Imageable(st.GetPrimAtPath(pth)).MakeVisible()
            out["mib_end"] = mib()
            return out

        return self.run(body, timeout=900.0)

    def kit_settings(self, keys: List[str],
                     values: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """The carb settings named, as the running kit has them (for diffing versions);
        `values` are set first."""
        self._require_booted()
        if self.mock:
            return {}

        def body() -> Dict[str, Any]:
            import carb
            cs = carb.settings.get_settings()
            for k, v in (values or {}).items():
                cs.set(k, v)
            return {k: cs.get(k) for k in list(keys) + list(values or {})}
        return self.run(body)

    def audit(self, action: str, targets: Optional[List[Dict[str, Any]]] = None,
              every_n: int = 4, shrink: float = 1.0) -> Dict[str, Any]:
        """A collision audit: does any arm or tool pass through anything, and when.

        The tools are referenced visual-only and the arms are position-driven, so a tool
        driven through a part or a fixture reports nothing - the only signal was a human
        watching the film. This asks PhysX every `every_n` steps which colliders each
        target's meshes overlap, using each mesh's own bound as an oriented box (a slight
        over-estimate at a cylinder's corners, so a hit is a candidate to look at, not a
        verdict). Hits inside the target's own root are ignored, as is anything under an
        `ignore` prefix.

        action: "start" (with targets [{"name", "root", "ignore": [...]}]), "read", "stop".
        """
        self._require_booted()
        if self.mock:
            return {"error": "mock"}
        if action == "start":
            return self.run(lambda: self._audit_start(targets or [], int(every_n),
                                                     float(shrink)))
        if action == "read":
            return self.run(self._audit_report)
        if action == "clock":
            # The audit's own clock: physics time summed since start, which a reset does
            # not rewind (sim_time does), so a hit can be put against the step it was in.
            return {"t": round(float((getattr(self, "_audit", None) or {}).get("t", 0.0)), 3)}
        if action == "stop":
            def _stop():
                try:
                    self.world.remove_physics_callback("collision_audit")
                except Exception:  # noqa: BLE001
                    pass
                out = self._audit_report()
                self._audit = None
                return out
            return self.run(_stop)
        raise ValueError("audit action is start, read or stop")

    def _audit_start(self, targets: List[Dict[str, Any]], every_n: int,
                     shrink: float) -> Dict[str, Any]:
        from pxr import Usd, UsdGeom

        stage = self._isaac.omni_usd.get_context().get_stage()
        meshes = []
        for t in targets:
            root = stage.GetPrimAtPath(str(t["root"]))
            if not root or not root.IsValid():
                continue
            for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
                if not prim.IsA(UsdGeom.Mesh):
                    continue
                if UsdGeom.Imageable(prim).ComputeVisibility() == UsdGeom.Tokens.invisible:
                    continue
                purpose = UsdGeom.Imageable(prim).ComputePurpose()
                if purpose not in (UsdGeom.Tokens.default_, UsdGeom.Tokens.render):
                    continue
                rng = UsdGeom.Boundable(prim).ComputeUntransformedBound(
                    Usd.TimeCode.Default(), UsdGeom.Tokens.default_,
                    UsdGeom.Tokens.render).ComputeAlignedRange()
                if rng.IsEmpty():
                    continue
                meshes.append({"target": str(t.get("name", t["root"])),
                               "root": str(t["root"]),
                               "ignore": [str(x) for x in (t.get("ignore") or [])],
                               "path": prim.GetPath(), "min": rng.GetMin(),
                               "max": rng.GetMax()})
        self._audit = {"meshes": meshes, "every_n": max(1, every_n), "step": 0,
                       "shrink": shrink, "hits": {}, "events": [], "checks": 0, "t": 0.0}
        try:
            self.world.remove_physics_callback("collision_audit")
        except Exception:  # noqa: BLE001
            pass
        self.world.add_physics_callback("collision_audit", self._audit_step)
        return {"meshes": len(meshes),
                "targets": sorted({m["target"] for m in meshes})}

    def _audit_step(self, step_size: float) -> None:
        a = getattr(self, "_audit", None)
        if not a:
            return
        a["step"] += 1
        a["t"] += float(step_size)
        if a["step"] % a["every_n"]:
            return
        try:
            self._audit_check(a)
        except Exception:  # noqa: BLE001
            LOGGER.exception("collision audit check failed")

    def _audit_check(self, a: Dict[str, Any]) -> None:
        import carb
        from omni.physx import get_physx_scene_query_interface
        from pxr import Gf, UsdGeom

        stage = self._isaac.omni_usd.get_context().get_stage()
        cache = UsdGeom.XformCache()
        query = get_physx_scene_query_interface()
        now = a["t"]
        a["checks"] += 1
        for m in a["meshes"]:
            prim = stage.GetPrimAtPath(m["path"])
            if not prim:
                continue
            mat = cache.GetLocalToWorldTransform(prim)
            lo, hi = m["min"], m["max"]
            centre = mat.Transform((lo + hi) / 2.0)
            axes = [Gf.Vec3d(mat.GetRow3(i)) for i in range(3)]
            scale = [ax.GetLength() for ax in axes]
            if min(scale) <= 0:
                continue
            half = [(hi[i] - lo[i]) / 2.0 * scale[i] * a["shrink"] for i in range(3)]
            if min(half) <= 1e-5:
                half = [max(h, 1e-4) for h in half]
            rot = Gf.Matrix3d(*(list(axes[0] / scale[0]) + list(axes[1] / scale[1])
                                + list(axes[2] / scale[2])))
            q = rot.GetOrthonormalized().ExtractRotation().GetQuat()
            im = q.GetImaginary()
            found = []

            def _report(hit, _found=found):
                _found.append(str(hit.collision))
                return True

            query.overlap_box(carb.Float3(*half), carb.Float3(*centre),
                              carb.Float4(im[0], im[1], im[2], q.GetReal()),
                              _report, False)
            for other in found:
                if other.startswith(m["root"]) or any(other.startswith(x)
                                                      for x in m["ignore"]):
                    continue
                key = (m["target"], str(m["path"]), other)
                h = a["hits"].get(key)
                if h is None:
                    a["hits"][key] = [now, now, 1]
                    if len(a["events"]) < 400:
                        a["events"].append([round(now, 2), m["target"],
                                            str(m["path"]), other])
                else:
                    if now - h[1] > 1.0 and len(a["events"]) < 400:
                        a["events"].append([round(now, 2), m["target"],
                                            str(m["path"]), other])
                    h[1], h[2] = now, h[2] + 1

    def _audit_report(self) -> Dict[str, Any]:
        a = getattr(self, "_audit", None)
        if not a:
            return {"running": False}
        return {"running": True, "checks": a["checks"], "meshes": len(a["meshes"]),
                "sim_time": round(float(self.world.current_time), 2),
                "hits": [[k[0], k[1], k[2], round(v[0], 2), round(v[1], 2), v[2]]
                         for k, v in sorted(a["hits"].items(), key=lambda kv: kv[1][0])],
                "events": list(a["events"])}

    def prim_masses(self, paths: List[str]) -> Dict[str, Any]:
        """What each prim weighs, as physx sees it.

        A position drive that cannot hold a pose is either weakly driven or heavily
        loaded, and the joint angles cannot tell you which. This is the other half of
        the answer.
        """
        self._require_booted()
        from pxr import UsdPhysics

        stage = self._isaac.omni_usd.get_context().get_stage()
        out: Dict[str, Any] = {}
        for path in paths:
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                out[path] = None
                continue
            entry: Dict[str, Any] = {}
            if prim.HasAPI(UsdPhysics.MassAPI):
                mass = UsdPhysics.MassAPI(prim)
                for label, attr in (("mass_kg", mass.GetMassAttr()),
                                    ("density", mass.GetDensityAttr())):
                    entry[label] = attr.Get() if attr and attr.IsValid() else None
                inertia = mass.GetDiagonalInertiaAttr()
                entry["inertia"] = (list(inertia.Get())
                                    if inertia and inertia.Get() is not None else None)
            else:
                entry["mass_kg"] = None
            out[path] = entry
        return out

    def prim_poses(self, paths: List[str]) -> Dict[str, Dict[str, Any]]:
        """World pose of arbitrary prims, for when config and reality disagree.

        Props are addressed by name; this takes prim paths, which is what you need when
        the question is about something the module authored rather than something the
        config named - a gripper's attachment point, an arm link, a joint. Positions are
        millimetres.
        """
        self._require_booted()
        if self.mock:
            return {}

        def _read() -> Dict[str, Dict[str, Any]]:
            # Read the pose straight out of USD. This used to build a
            # `SingleXFormPrim` per path, which is a physics object: constructing one on
            # a prim INSIDE an articulation re-registers it with isaac's core and takes
            # that articulation's view out from under the handle holding it. The arm
            # then fails its NEXT command with "Failed to get DOF positions from
            # backend" - so simply ASKING where a link or a tool was became a way to
            # break the arm, and it read as an unrelated fault every time.
            from pxr import Usd, UsdGeom

            stage = self._isaac.omni_usd.get_context().get_stage()
            out: Dict[str, Dict[str, Any]] = {}
            for path in paths:
                prim = stage.GetPrimAtPath(path)
                if not prim or not prim.IsValid():
                    out[path] = {"error": f"no prim at {path}"}
                    continue
                matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default())
                translation = matrix.ExtractTranslation()
                quat = matrix.ExtractRotationQuat()
                imaginary = quat.GetImaginary()
                out[path] = {
                    "position_mm": [float(v) * 1000.0 for v in translation],
                    "orientation_wxyz": [float(quat.GetReal()),
                                         float(imaginary[0]), float(imaginary[1]),
                                         float(imaginary[2])],
                    # The prim's own axes in world terms, so a caller can see which way
                    # a tool actually points rather than assuming the convention.
                    "x_axis": [float(v) for v in matrix.GetRow3(0).GetNormalized()],
                    "y_axis": [float(v) for v in matrix.GetRow3(1).GetNormalized()],
                    "z_axis": [float(v) for v in matrix.GetRow3(2).GetNormalized()],
                }
            return out

        return self.run(_read)

    def status(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "booted": self._booted.is_set(),
            "mock": self.mock,
            "error": str(self._boot_error) if self._boot_error else "",
        }
        if self._booted.is_set() and not self.mock:
            out["playing"] = self.run(lambda: bool(self.world.is_playing()))
            out["sim_time"] = self.run(lambda: float(self.world.current_time))
            out["loop_s"] = dict(self._loop_s)
        if self._booted.is_set():
            out["sim_clock"] = self.run(lambda: float(self._sim_clock))
            ka = getattr(self, "_keepalive", None)
            if ka is not None:
                out["render_keepalive"] = self.run(
                    lambda: bool(ka._render_product.hydra_texture.get_updates_enabled()))
        if self._static_notes:
            out["static"] = {path: {"count": len(prims), "prims": prims[:6]}
                             for path, prims in self._static_notes.items()}
        return out

    def add_usd_reference(
        self,
        usd_path: str,
        prim_path: str,
        position: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        self._require_booted()
        if self.mock:
            return

        def _add():
            self._isaac.add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
            prim = self._isaac.SingleXFormPrim(prim_path)
            prim.set_world_pose(position=list(position))

        self.run(_add, timeout=60.0)

    # ------------------------------------------------------------------
    # component factories
    # ------------------------------------------------------------------

    # attributes that only affect the viam-side model, not the spawned prim
    # Attributes that can be changed on a running sim, because they do not describe how
    # anything was spawned. The drive gains and the joint speed belong here: they are
    # properties of the controller, not of the prim, and treating them as spawn config
    # meant a pushed change was only WARNED about - "restart the module to apply" - and
    # then silently ignored. Measured, that cost a whole teaching run: damping 300 was
    # pushed, the module restarted before viam-server had the new config, the old 150 was
    # applied, and the reconfigure that followed changed nothing.
    _RUNTIME_KEYS = frozenset(
        {"world", "move_timeout_sec", "max_linear_mps", "max_angular_rps",
         "joint_speed_deg_s", "joint_accel_deg_s2", "home_joints_deg",
         "drive_stiffness", "drive_damping", "drive_damping_ratio",
         "drive_max_effort"}
    )

    def _cached_handle(
        self, kind: str, name: str, attrs: Dict[str, Any], factory: Callable[[], Any]
    ) -> Any:
        if name in self._handles:
            old_attrs, handle = self._handles[name]
            strip = lambda a: {k: v for k, v in a.items() if k not in self._RUNTIME_KEYS}
            if strip(old_attrs) != strip(attrs):
                LOGGER.warning(
                    "%s %r: spawn config changed but the prim is already in the "
                    "stage; restart the module to apply",
                    kind,
                    name,
                )
            return handle
        handle = factory()
        self._handles[name] = (dict(attrs), handle)
        return handle

    def _usd_exists(self, path: str) -> Optional[bool]:
        """True/False if we can check, None if omni.client is unavailable."""
        client = getattr(self._isaac, "client", None)
        if client is None:
            return None
        try:
            result, _ = client.stat(path)
            return result == client.Result.OK
        except Exception:
            return None

    def _resolve_usd(self, attrs: Dict[str, Any]) -> Tuple[Optional[str], Dict[str, Any]]:
        """Return (absolute usd path or None, known-asset metadata)."""
        meta: Dict[str, Any] = {}
        usd = attrs.get("usd_path")
        asset = attrs.get("asset")
        if asset:
            if asset not in KNOWN_ASSETS:
                raise ValueError(
                    f"unknown asset {asset!r}; known: {sorted(KNOWN_ASSETS)} "
                    "(or set usd_path directly)"
                )
            meta = KNOWN_ASSETS[asset]
            if not usd:
                root = self._isaac.get_assets_root_path()
                if root is None:
                    raise RuntimeError("could not reach the isaac sim assets server")
                candidates = meta["usd"]
                for rel in candidates:
                    if self._usd_exists(root + rel) is not False:
                        usd = root + rel
                        break
                if usd is None:
                    raise ValueError(
                        f"asset {asset!r}: none of {candidates} exist under {root}; "
                        "the asset layout may have changed in this isaac release"
                    )
        # a USD reference to a missing file "succeeds" but leaves an empty
        # prim, which later fails with confusing physics-tensor errors -
        # catch it here instead
        if usd and self._usd_exists(usd) is False:
            raise ValueError(f"usd not found: {usd}")
        return usd, meta

    def _retry_pending_lamps(self) -> None:
        """Place any lamp whose parent prim has since appeared."""
        if not getattr(self, "_pending_lamps", None):
            return
        try:
            stage = self._isaac.omni_usd.get_context().get_stage()
            self._apply_lamps(stage, list(self._pending_lamps))
        except Exception:  # noqa: BLE001
            LOGGER.exception("failed to place pending lamps")

    def create_arm(self, name: str, attrs: Dict[str, Any]) -> "ArmHandle":
        self._require_booted()
        if self.mock:
            factory = lambda: MockArmHandle(name, attrs)
        else:
            factory = lambda: self.run(
                lambda: self._create_arm_isaac(name, attrs), timeout=120.0
            )
        existing = name in self._handles
        handle = self._cached_handle("arm", name, attrs, factory)
        if existing and not self.mock:
            # Re-apply the gains a reconfigure may have changed, and remember the attrs
            # they came from, so pushing a gain change takes effect without a restart.
            art = getattr(handle, "_art", None)
            if art is not None:
                self.run(lambda: self._set_arm_gains(name, art, attrs))
            self._handles[name] = (dict(attrs), handle)
        if not self.mock:
            # The arm's links are in the stage now, so a flash that wanted one can be hung.
            self.run(self._retry_pending_lamps)
        return handle

    def _create_arm_isaac(self, name: str, attrs: Dict[str, Any]) -> "IsaacArmHandle":
        from .spatial import quat_mul, to_vec3

        usd, meta = self._resolve_usd(attrs)
        prim_path = attrs.get("prim_path") or f"/World/{_prim_name(name)}"
        if usd:
            self._isaac.add_reference_to_stage(usd_path=usd, prim_path=prim_path)
        # Tooling with a COLLIDER has to exist before the articulation does. Adding a
        # collider under a link afterwards changes the articulation's collision set, and
        # the physics views built for the arm no longer describe it: every later command
        # on that arm fails with "Failed to get DOF positions from backend". Resetting
        # the world afterwards does not repair it, because the handle holds the view it
        # was built with. The cup's VISUAL never hit this - it has no collider.
        if attrs.get("hook"):
            stage = self._isaac.omni_usd.get_context().get_stage()
            self._author_tool_hook(stage, name, attrs["hook"])

        # The drive force ceiling has to be written to USD, not asked for through the
        # articulation. `set_max_efforts` accepts the request and does not apply it (see
        # `_set_drive_gains`), so the arm spent its whole tuning against the asset's
        # authored [150,150,150,28,28,28]. PhysX reads the ceiling from each joint's
        # angular DriveAPI, and writing it here - before `SingleArticulation` is built -
        # is the one place the value is still the articulation's to take.
        if attrs.get("drive_max_effort") is not None:
            stage = self._isaac.omni_usd.get_context().get_stage()
            self._author_drive_max_force(
                stage, name, prim_path, attrs["drive_max_effort"])

        # Set the base pose directly on the USD prim. The `position=` /
        # `orientation=` kwargs on SingleArticulation don't persist across the
        # world.reset() below when another articulation is already in the
        # scene, so every arm silently ends up at the world origin and their
        # bodies penetrate. Writing to USD persists across resets.
        position = to_vec3(attrs.get("position"))
        # The frame config orients the arm's *kinematic base*, which for some assets is
        # not the USD's root prim (see _UR_BASE_ROTATION_WXYZ). Fold the asset's fixed
        # offset in here, on the isaac side only: the frame system must keep describing
        # the base the kinematics file describes, or the planner and the simulation stop
        # agreeing about which way the arm faces.
        orientation = attrs.get("orientation_wxyz")
        orientation = (
            tuple(float(v) for v in orientation) if orientation is not None
            else (1.0, 0.0, 0.0, 0.0)
        )
        base_rotation = meta.get("base_rotation_wxyz")
        if attrs.get("ignore_base_rotation"):
            base_rotation = None
        if base_rotation is not None:
            orientation = quat_mul(orientation, tuple(float(v) for v in base_rotation))

        # Write the pose onto the USD prim rather than passing it to
        # SingleArticulation: those kwargs do not survive the world.reset() below once
        # another articulation is in the scene, so every arm silently lands on the origin
        # and the bodies interpenetrate. USD state persists across resets.
        pose_kwargs: Dict[str, Any] = {"position": list(position)}
        if orientation != (1.0, 0.0, 0.0, 0.0):
            pose_kwargs["orientation"] = list(orientation)
        try:
            self._isaac.SingleXFormPrim(prim_path).set_world_pose(**pose_kwargs)
        except Exception:
            LOGGER.exception("failed to set base pose for %s", name)

        art = self._isaac.SingleArticulation(prim_path=prim_path, name=name)
        self.world.scene.add(art)
        self.world.reset()
        self._set_arm_gains(name, art, attrs)
        home = attrs.get("home_joints_deg")
        if home:
            art.set_joint_positions(_as_radians(home))
            LOGGER.info("arm %s starts at its home configuration %s", name,
                        [round(float(v), 1) for v in home])

        ee = None
        ee_path = attrs.get("end_effector_prim")
        if ee_path:
            ee = self._isaac.SingleXFormPrim(ee_path)
        return IsaacArmHandle(self, art, ee, name)

    def _set_arm_gains(self, name: str, art, attrs: Dict[str, Any]) -> None:
        """Report the articulation's drive gains, and raise them if asked.

        An arm whose position drives cannot hold it against gravity sags away from the
        pose it was commanded to and reports the joints that could not get there. That is
        the same report a collision produces, which is why this went a long way
        undiagnosed: measured, the elbow sat 34 degrees below its target while the three
        wrist joints - which carry almost nothing - were exact to a tenth of a degree.

        The gains come from whatever authored the USD, which has no way to know what the
        arm will be asked to carry. Measured on this cell's UR5e, stiffness is around
        5e5 on every joint while damping reads [21.7, 23.6, 234.5, 90.4, 3.5, 0.2] - four
        orders of magnitude short of critical, and 0.2 on the wrist roll. A drive that
        stiff and that undamped does not settle; it rings, and a joint still ringing when
        the tolerance is checked is indistinguishable from one held by a collision.

        `drive_damping_ratio` sets damping as a fraction of each joint's own stiffness,
        which travels between assets in a way an absolute number does not.
        `drive_stiffness` and `drive_damping` set absolute values, scalar or per joint.
        """
        controller = art.get_articulation_controller()
        gains = controller.get_gains()
        efforts = None
        for getter in ("get_max_efforts", "get_applied_action"):
            try:
                efforts = getattr(controller, getter)()
                break
            except Exception:  # noqa: BLE001
                continue
        LOGGER.info("arm %s drive gains: stiffness=%s damping=%s max_effort=%s", name,
                    _as_list(gains[0]), _as_list(gains[1]), _as_list(efforts))

        stiffness = attrs.get("drive_stiffness")
        damping = attrs.get("drive_damping")
        ratio = attrs.get("drive_damping_ratio")
        if stiffness is None and damping is None and ratio is None:
            return
        import numpy as np

        dof = art.num_dof

        def spread(value, fallback):
            if value is None:
                return np.asarray(fallback, dtype=float)[:dof]
            if isinstance(value, (list, tuple)):
                return np.asarray([float(v) for v in value], dtype=float)[:dof]
            return np.full(dof, float(value))

        kps = spread(stiffness, gains[0])
        if ratio is not None and damping is None:
            kds = kps * float(ratio)
        else:
            kds = spread(damping, gains[1])
        controller.set_gains(kps=kps, kds=kds)

        # A position drive that cannot produce the torque simply sits short of its
        # target, by a degree or two, at exactly the extended and loaded poses where the
        # gravity moment is largest - which reads as a stuck joint a hair from home.
        # Asked for, and whether it took. Measured on this cell, drive_max_effort was
        # set to 300 and the limits still read the asset's authored [150,150,150,28,28,
        # 28] afterwards - so the whole time the arm was being tuned against a ceiling
        # that had not moved, and the only record of the failure went to a log this
        # cell sends to /dev/null. Config that silently does not apply is worse than
        # config that is refused, so the request and its outcome are kept where
        # `efforts` can report them.
        effort = attrs.get("drive_max_effort")
        note: Dict[str, Any] = {"requested_max_effort": effort}
        if effort is not None:
            wanted = spread(effort, [0.0] * dof)
            for setter, target in (("set_max_efforts", controller),
                                   ("set_max_efforts", art),
                                   ("set_max_joint_efforts", art)):
                fn = getattr(target, setter, None)
                if fn is None:
                    continue
                try:
                    fn(wanted)
                except Exception as exc:  # noqa: BLE001
                    note.setdefault("refused_by", []).append(
                        f"{type(target).__name__}.{setter}: {exc}")
                    continue
                note["set_by"] = f"{type(target).__name__}.{setter}"
                break
            LOGGER.info("arm %s max effort request: %s", name, note)
        self._gain_notes[name] = note
        LOGGER.info("arm %s drive gains set to stiffness=%s damping=%s",
                    name, _as_list(kps), _as_list(kds))

    def create_gripper(self, name: str, attrs: Dict[str, Any]) -> "GripperHandle":
        self._require_booted()
        if self.mock:
            factory = lambda: MockGripperHandle(name, attrs)  # noqa: E731
        else:
            factory = lambda: self.run(  # noqa: E731
                lambda: self._create_gripper_isaac(name, attrs), timeout=120.0
            )
        return self._cached_handle("gripper", name, attrs, factory)

    def _create_gripper_isaac(self, name: str, attrs: Dict[str, Any]) -> "IsaacGripperHandle":
        """Author the suction rig through the shared surface_gripper module.

        The authoring itself lives in `surface_gripper.py`, vendored from
        DTCurrie/viam-isaac-sim so the two trees carry one implementation rather than two
        to reconcile. This method's job is only to translate *this* module's config into
        the frame that module expects.

        That translation is the whole subtlety. It wants a tool frame whose **+Z is the
        cup axis**; our `offset` is in the parent prim's frame, where isaac's UR flange
        puts the tool along **+x**. Same cup, two conventions - getting it wrong aims the
        plugin's raycast sideways, and it then grips only when geometry happens to line
        up, which is exactly the intermittency this replaced.
        """
        import math

        from .asset_catalog import CUP_APPROACH_GAP_MM
        from .spatial import _cross, _dot, _norm, quat_from_axis_angle, quat_mul
        from .surface_gripper import author_attachment_rig, gripper_limits

        stage = self._isaac.omni_usd.get_context().get_stage()
        parent = attrs.get("parent_prim")
        if not parent:
            raise ValueError(
                f"gripper {name}: needs parent_prim, the prim it hangs off "
                "(e.g. an arm's flange)"
            )
        parent_prim = stage.GetPrimAtPath(parent)
        if not parent_prim.IsValid():
            raise ValueError(f"gripper {name}: parent_prim {parent!r} is not in the stage")
        # Everything below builds along the parent's +x, the UR flange convention. A
        # flange whose tool runs along its +Z instead (the uf850's link6, which is viam's
        # end-effector frame) gets a child frame turned -90 deg about Y, so that child's
        # +x is the flange's +Z, and the rig hangs off the child. `offset` is then in the
        # child's frame: [d, 0, 0] still means d metres down the tool axis.
        if str(attrs.get("parent_tool_axis", "x")).lower() == "z":
            from pxr import Gf, UsdGeom

            tool_frame = f"{parent}/tool_x"
            xf = UsdGeom.Xform.Define(stage, tool_frame)
            xf.ClearXformOpOrder()
            half = math.sqrt(0.5)
            xf.AddOrientOp().Set(Gf.Quatf(half, Gf.Vec3f(0.0, -half, 0.0)))
            parent, parent_prim = tool_frame, xf.GetPrim()

        from pxr import UsdGeom, UsdPhysics

        # The joints must hang from a rigid body, and the rig's scope must sit OUTSIDE
        # the articulation - a rigid body nested under a link is an error. So walk up to
        # the body for the anchor, and put the scope at world level.
        body_prim = parent_prim
        while body_prim.IsValid() and not body_prim.HasAPI(UsdPhysics.RigidBodyAPI):
            body_prim = body_prim.GetParent()
        if not body_prim.IsValid():
            raise ValueError(
                f"gripper {name}: no rigid body at or above {parent!r}; suction needs a "
                "body to pull against"
            )
        body_path = body_prim.GetPath().pathString

        cache = UsdGeom.XformCache()
        to_body = (cache.GetLocalToWorldTransform(parent_prim)
                   * cache.GetLocalToWorldTransform(body_prim).GetInverse())
        offset = [float(v) for v in (attrs.get("offset") or (0.0, 0.0, 0.0))]
        from pxr import Gf as _Gf

        tip = to_body.Transform(_Gf.Vec3d(*offset))
        tool_pos = (float(tip[0]), float(tip[1]), float(tip[2]))

        # Where the cup points, in the body's frame: the offset direction carried through
        # the same transform. Then the rotation that takes +Z onto it.
        direction = to_body.TransformDir(_Gf.Vec3d(*offset))
        direction = (float(direction[0]), float(direction[1]), float(direction[2]))
        length = _norm(direction)
        if length < 1e-9:
            raise ValueError(
                f"gripper {name}: offset is zero, so the cup has no axis to point along"
            )
        direction = tuple(v / length for v in direction)
        z_axis = (0.0, 0.0, 1.0)
        dot = max(-1.0, min(1.0, _dot(z_axis, direction)))
        if dot > 1.0 - 1e-9:
            tool_quat = (1.0, 0.0, 0.0, 0.0)
        elif dot < -1.0 + 1e-9:
            tool_quat = quat_from_axis_angle((1.0, 0.0, 0.0), math.pi)
        else:
            tool_quat = quat_from_axis_angle(_cross(z_axis, direction), math.acos(dot))

        position, orientation = self._isaac.SingleXFormPrim(body_path).get_world_pose()
        body_pose = (tuple(float(v) for v in position),
                     tuple(float(v) for v in orientation))

        limits = gripper_limits(attrs)

        rig = author_attachment_rig(
            self._isaac.gripper_modules,
            stage,
            scope_path=f"/World/{_prim_name(name)}_rig",
            body0_path=body_path,
            body0_world_pose=body_pose,
            tool_pose_in_body0=(tool_pos, tool_quat),
            points_tool_m=[(0.0, 0.0, 0.0)],
            clearance_offset_m=float(
                attrs.get("clearance_offset_m", CUP_APPROACH_GAP_MM / 1000.0)),
            limits=limits,
            compliance=None,  # one cup: lock every axis, which welds it to the payload
        )

        # The plugin only looks for grippers on the first physics frame after play, so a
        # rig authored after the last reset is never seen.
        self.world.reset()

        self._author_gripper_visual(stage, parent, name, offset, attrs)


        view = self._isaac.GripperView(paths=rig.gripper_path)
        # The import report matters: without PhysxSchema the rig's ghost anchor keeps
        # its gravity, and an anchor that falls while welded to the wrist is a wrist
        # that cannot turn.
        LOGGER.info("gripper %s: rig %s on body %s, cup axis %s in its frame; isaac "
                    "reports %s", name, rig.scope_path, body_path,
                    [round(v, 3) for v in direction],
                    self._isaac.gripper_modules.get("report"))
        # Visible jaws, unless the config asks for none. A gripper that cannot be seen
        # gripping cannot be reviewed by watching the cell work, and watching it is how
        # several of this build's worst faults were found.
        jaw = attrs.get("jaw")
        jaw_prim = None
        if jaw is not False:
            spec = dict(jaw or {})
            spec.setdefault("reach_mm", float(attrs.get("offset", [0.12])[0]) * 1000.0)
            try:
                self.author_jaw(parent, spec)
                jaw_prim = parent
            except Exception:                                      # noqa: BLE001
                LOGGER.exception("gripper %s: could not author visible jaws", name)
        return IsaacGripperHandle(self, view, rig.gripper_path, jaw_prim)

    def _author_gripper_visual(
        self, stage: Any, parent_path: str, name: str,
        offset: List[float], attrs: Dict[str, Any],
    ) -> None:
        """Give the tool something to look at.

        The cup exists to the planner (a capsule in world_state) and to physics (the
        attachment rig), but nothing renders it, so the arm appears to pick things up
        with nothing on the end of it - which is exactly the kind of gap watching the
        video catches and a probe never does.

        Visual only: no CollisionAPI. A second collider on the tool would fight the one
        the planner already reasons about, and would be the arm colliding with itself.
        """
        import math

        from pxr import Gf, UsdGeom

        if attrs.get("visual") is False:
            return
        spec = attrs.get("geometry") or {}
        radius = float(spec.get("radius_mm", 25.0)) / 1000.0
        length = math.sqrt(sum(v * v for v in offset)) or 0.12

        root_path = f"{parent_path}/{_prim_name(name)}_visual"
        root = UsdGeom.Xform.Define(stage, root_path)
        # Point the visual down the same axis the cup acts along rather than assuming
        # one: `offset` is in this prim's frame and is the only statement of it. The
        # tubes are built along +Z, so this is the rotation taking +Z onto that.
        direction = [v / length for v in offset] if length else [0.0, 0.0, 1.0]
        dot = max(-1.0, min(1.0, direction[2]))
        if dot > 1.0 - 1e-9:
            quat = Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0))
        elif dot < -1.0 + 1e-9:
            quat = Gf.Quatf(0.0, Gf.Vec3f(1.0, 0.0, 0.0))
        else:
            axis = Gf.Vec3f(-direction[1], direction[0], 0.0).GetNormalized()
            half = math.acos(dot) / 2.0
            quat = Gf.Quatf(math.cos(half), axis * math.sin(half))
        UsdGeom.Xformable(root.GetPrim()).AddOrientOp().Set(quat)

        def tube(suffix, r, h, z, colour):
            prim = UsdGeom.Cylinder.Define(stage, f"{root_path}/{suffix}")
            prim.CreateRadiusAttr(r)
            prim.CreateHeightAttr(h)
            prim.CreateAxisAttr("Z")
            prim.CreateExtentAttr([Gf.Vec3f(-r, -r, -h / 2), Gf.Vec3f(r, r, h / 2)])
            UsdGeom.Xformable(prim.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0, 0, z))
            prim.CreateDisplayColorAttr([Gf.Vec3f(*colour)])

        # A vacuum tool: a coupling at the flange, a stem, and the cup at the tip.
        body = length * 0.62
        tube("coupling", radius * 1.25, length * 0.14, length * 0.07, (0.26, 0.27, 0.30))
        tube("stem", radius * 0.55, body, length * 0.14 + body / 2.0, (0.72, 0.73, 0.75))
        tube("cup", radius, length * 0.22, length - length * 0.11, (0.12, 0.12, 0.14))

    def _author_drive_max_force(
        self, stage: Any, name: str, prim_path: str, effort: Any
    ) -> None:
        """Raise each joint's angular drive force ceiling, in USD, where PhysX reads it.

        Written rather than requested, and the outcome recorded. Both halves are the
        lesson from `drive_max_effort`: the articulation API took a request for 300 N.m
        and left the limits at the asset's authored values, and the only record of that
        went to a log this cell sends to /dev/null. So this reports the joints it
        actually wrote and the values they held before, and `efforts` surfaces it.

        A ceiling is permission to pull harder, not an instruction to. A joint that was
        never near its limit moves exactly as it did.
        """
        from pxr import UsdPhysics

        root = stage.GetPrimAtPath(prim_path)
        note: Dict[str, Any] = {"requested_drive_max_force": effort}
        if not root or not root.IsValid():
            note["refused"] = f"no prim at {prim_path}"
            self._drive_force_notes[name] = note
            return

        joints = [prim for prim in stage.Traverse()
                  if str(prim.GetPath()).startswith(prim_path + "/")
                  and prim.IsA(UsdPhysics.RevoluteJoint)]
        joints.sort(key=lambda prim: str(prim.GetPath()))

        # A scalar for every joint, or a MAPPING BY JOINT NAME. Never a list.
        #
        # This runs before the articulation is built, so `dof_names` does not exist yet and
        # the only order available is the prim paths' own - which is alphabetical, and for a
        # ur5e that is elbow, shoulder_lift, shoulder_pan, wrist_1, wrist_2, wrist_3. The
        # dof order is shoulder_pan, shoulder_lift, elbow, ... So a per-joint LIST, written
        # in the order anyone would write it, lands its first three values on the third,
        # second and first joints. It would not fail; it would quietly give the elbow the
        # base's ceiling and the base the elbow's, and the symptom would be an arm that
        # stalls in one configuration and not another.
        #
        # Nothing in this repo passes a list - the fragments all pass a scalar - so this
        # was latent. It is refused rather than guessed at, and a dict keyed by joint name
        # is accepted instead, which is how `joint_limits_deg` is already written and is
        # order-independent by construction. (Found by the xarm6 session, who hit the same
        # ordering in a helper of their own.)
        by_name: Dict[str, float] = {}
        if isinstance(effort, dict):
            by_name = {str(k): float(v) for k, v in effort.items()}
            known = {prim.GetName() for prim in joints}
            unknown = sorted(set(by_name) - known)
            if unknown:
                raise ValueError(
                    f"arm {name}: drive_max_effort names {unknown} which are not joints "
                    f"of this arm; it has {sorted(known)}")
            wanted = [by_name.get(prim.GetName()) for prim in joints]
        elif isinstance(effort, (list, tuple)):
            raise ValueError(
                f"arm {name}: drive_max_effort must be a number for every joint, or a "
                f"map of joint name to number - not a list. This is authored before the "
                f"articulation exists, so the only order available is the prim paths' "
                f"(alphabetical), which is not the dof order a list would be written in: "
                f"for a ur5e that is elbow, shoulder_lift, shoulder_pan, ... against a dof "
                f"order of shoulder_pan, shoulder_lift, elbow, ... A list would silently "
                f"land on the wrong joints.")
        else:
            wanted = [float(effort)] * len(joints)

        written: Dict[str, Any] = {}
        for prim, value in zip(joints, wanted):
            if value is None:        # a name-keyed map need not mention every joint
                continue
            drive = UsdPhysics.DriveAPI.Get(prim, "angular")
            if not drive:
                drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            attr = drive.GetMaxForceAttr() or drive.CreateMaxForceAttr()
            was = attr.Get()
            attr.Set(float(value))
            written[prim.GetName()] = {"was": None if was is None else round(float(was), 1),
                                       "now": round(float(value), 1)}
        note["joints"] = written
        if not written:
            note["refused"] = f"no revolute joints under {prim_path}"
        LOGGER.info("arm %s drive max force authored: %s", name, note)
        self._drive_force_notes[name] = note

    def _author_tool_hook(self, stage: Any, name: str, spec: Dict[str, Any]) -> None:
        """A small hook on the tool plate, for opening the mailbox door.

        A suction cup is the wrong tool for this door, and the geometry says so: the door
        is bottom-hinged and opens OUTWARD only, so it has to be pulled, and pulling a
        flat panel needs a seal the cup cannot reliably get on a collider standing 0.5 mm
        proud of the shell's hull behind it.

        The door does carry something made for pulling. `Door/Collision/Handle` is a
        separate collider standing 22 mm proud of the panel, 200 mm up from the hinge -
        a handle. So the tool gets a finger that reaches past the cup, goes under that
        bar, and pulls.

        Carried BESIDE the cup rather than in front of it, so the pick is untouched: the
        cup still acts along its own axis with nothing in the way.

        This one has a CollisionAPI, unlike the cup's visual - it exists to move real
        geometry - so it is authored before the articulation is built, and the planner is
        told about it separately by `qc/round.hook_transform`.
        """
        import math

        from pxr import Gf, UsdGeom, UsdPhysics

        parent_path = str(spec["parent_prim"])
        reach = float(spec.get("reach_mm", 150.0)) / 1000.0
        finger = float(spec.get("finger_mm", 45.0)) / 1000.0
        side = float(spec.get("side_mm", 70.0)) / 1000.0
        thick = float(spec.get("thickness_mm", 12.0)) / 1000.0
        axis = [float(v) for v in (spec.get("axis") or (1.0, 0.0, 0.0))]
        norm = math.sqrt(sum(v * v for v in axis)) or 1.0
        axis = [v / norm for v in axis]
        if not stage.GetPrimAtPath(parent_path).IsValid():
            raise ValueError(f"hook for {name}: no prim at {parent_path}")

        root_path = f"{parent_path}/{_prim_name(name)}_hook"
        UsdGeom.Xform.Define(stage, root_path)

        def bar(suffix, size, centre, colour):
            prim = UsdGeom.Cube.Define(stage, f"{root_path}/{suffix}")
            prim.CreateSizeAttr(1.0)
            # A Cube's extent does not follow its scale, and a stale extent is what makes
            # a scaled box report the wrong bounds.
            prim.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
            x = UsdGeom.Xformable(prim.GetPrim())
            x.AddTranslateOp().Set(Gf.Vec3d(*centre))
            x.AddScaleOp().Set(Gf.Vec3f(*size))
            prim.CreateDisplayColorAttr([Gf.Vec3f(*colour)])
            UsdPhysics.CollisionAPI.Apply(prim.GetPrim())
            UsdPhysics.MeshCollisionAPI.Apply(
                prim.GetPrim()).CreateApproximationAttr("boundingCube")

        across = [0.0, 0.0, 1.0] if abs(axis[2]) < 0.9 else [0.0, 1.0, 0.0]
        perp = [axis[1] * across[2] - axis[2] * across[1],
                axis[2] * across[0] - axis[0] * across[2],
                axis[0] * across[1] - axis[1] * across[0]]
        pnorm = math.sqrt(sum(v * v for v in perp)) or 1.0
        perp = [v / pnorm for v in perp]

        bar("stem",
            [abs(axis[i]) * reach + thick * (1 - abs(axis[i])) for i in range(3)],
            [axis[i] * (reach / 2.0) + perp[i] * side for i in range(3)],
            (0.85, 0.55, 0.10))
        bar("finger",
            [abs(perp[i]) * finger + thick * (1 - abs(perp[i])) for i in range(3)],
            [axis[i] * reach + perp[i] * (side - finger / 2.0) for i in range(3)],
            (0.95, 0.75, 0.15))
        LOGGER.info("arm %s: hook reach %.0f mm, finger %.0f mm, %.0f mm to the side "
                    "of %s", name, reach * 1000, finger * 1000, side * 1000, parent_path)

    def _keepalive_texture(self):
        """The 64x64 render product that keeps the renderer from ever going idle."""
        if getattr(self, "_keepalive", None) is None:
            import numpy as np

            cam = self._isaac.Camera(prim_path="/World/_render_keepalive", resolution=(64, 64),
                                     position=np.array([0.0, 0.0, -50.0]))
            cam.initialize(attach_rgb_annotator=False)
            self._keepalive = cam
        return self._keepalive._render_product.hydra_texture

    def _render_handoff(self, handle, tex, live: bool) -> None:
        """Turn one camera's render product on or off without the renderer ever rendering
        nothing: going from nothing back to something is what leaks 205-215 MB (see
        IsaacCameraHandle). On the sim thread, so no frame renders between the two."""
        keep = self._keepalive_texture()
        if live:
            tex.set_updates_enabled(True)
            keep.set_updates_enabled(False)
            return
        others = [h for h in self._camera_handles
                  if h is not handle and getattr(h, "_live", True)]
        if not others:
            keep.set_updates_enabled(True)
        tex.set_updates_enabled(False)

    def create_camera(self, name: str, attrs: Dict[str, Any]) -> "CameraHandle":
        self._require_booted()
        if self.mock:
            factory = lambda: MockCameraHandle(name, attrs)
        else:
            def factory():
                handle = self.run(lambda: self._create_camera_isaac(name, attrs),
                                  timeout=120.0)
                # The camera's BODY, where it has one: a referenced asset hung visual-only
                # on the same parent, the way a gripper's is (`_author_tool_asset`).
                # Outside the run above, because that call schedules its own.
                if attrs.get("tool") and attrs.get("parent_prim"):
                    self._author_tool_asset(str(attrs["parent_prim"]), dict(attrs["tool"]))
                return handle
        return self._cached_handle("camera", name, attrs, factory)

    def _create_camera_isaac(self, name: str, attrs: Dict[str, Any]) -> "IsaacCameraHandle":
        import math

        from .spatial import quat_from_euler_deg, to_vec3

        parent = attrs.get("parent_prim")
        if parent:
            self._require_prim(parent)
            prim_path = f"{parent.rstrip('/')}/{_prim_name(name)}"
        else:
            prim_path = attrs.get("prim_path") or f"/World/{_prim_name(name)}"
        width = int(attrs.get("width", 640))
        height = int(attrs.get("height", 480))

        kwargs: Dict[str, Any] = dict(
            prim_path=prim_path,
            name=name,
            resolution=(width, height),
        )
        if attrs.get("position") is not None:
            kwargs["position"] = list(to_vec3(attrs.get("position")))
        if attrs.get("orientation_rpy_deg") is not None:
            r, p, y = to_vec3(attrs.get("orientation_rpy_deg"))
            kwargs["orientation"] = list(quat_from_euler_deg(r, p, y))

        cam = self._isaac.Camera(**kwargs)
        cam.initialize()
        if attrs.get("depth"):
            # Depth along the optical axis (distance to the image plane), for
            # `get_point_cloud`. Rendered with the colour, so on demand like it.
            cam.add_distance_to_image_plane_to_frame()

        if parent:
            # camera rides a (possibly moving) link; pose is local to it.
            # default: small standoff along the link, looking out the +Z
            # (tool) axis - 180deg about X flips the usd camera's -Z forward.
            local_pos = to_vec3(attrs.get("local_position"), default=(0.0, 0.0, 0.05))
            r, p, y = to_vec3(
                attrs.get("local_orientation_rpy_deg"), default=(180.0, 0.0, 0.0)
            )
            quat = quat_from_euler_deg(r, p, y)
            cam.set_local_pose(list(local_pos), list(quat), camera_axes="usd")
        elif attrs.get("target") is not None:
            # aim at a target point (world axes: +X forward, +Z up)
            from .spatial import look_at_quat

            position = to_vec3(attrs.get("position"), default=(3.0, 3.0, 2.5))
            quat = look_at_quat(position, to_vec3(attrs.get("target")))
            cam.set_world_pose(list(position), list(quat), camera_axes="world")
        elif attrs.get("orientation_wxyz") is not None:
            position = to_vec3(attrs.get("position"))
            quat = tuple(float(v) for v in attrs["orientation_wxyz"])
            cam.set_world_pose(list(position), list(quat), camera_axes="world")

        # newly created cameras default to a 70 degree horizontal FOV - the
        # usd default lens is ~24 degrees, which reads as "zoomed way in".
        # computed via the aperture so usd unit conventions cancel out.
        if not attrs.get("prim_path"):
            fov = float(attrs.get("fov_deg", 70.0))
            aperture = cam.get_horizontal_aperture()
            cam.set_focal_length(aperture / (2.0 * math.tan(math.radians(fov) / 2.0)))
        elif attrs.get("fov_deg"):
            fov = float(attrs["fov_deg"])
            aperture = cam.get_horizontal_aperture()
            cam.set_focal_length(aperture / (2.0 * math.tan(math.radians(fov) / 2.0)))

        # Always set the clipping range, because the default is wrong for any camera
        # looking at something within arm's reach: USD's near plane is 1.0 world unit,
        # which on a metres stage is a metre. An inspection camera 780 mm from the part
        # renders the part - and every nearby link - as pure black, while the wall two
        # and a half metres behind comes out fine. It reads as a lighting or material
        # fault, and the frames it produces are exactly the frames a vision model would
        # be trained and scored on.
        near, far = [float(v) for v in
                     (attrs.get("clipping_range_m") or DEFAULT_CLIPPING_RANGE_M)]
        cam.set_clipping_range(near, far)
        fisheye = attrs.get("fisheye")
        if fisheye:
            self._author_fisheye(prim_path, fisheye, width, height)
        # Rendered only when read, unless the config says always (the film's overview).
        handle = IsaacCameraHandle(self, cam,
                                   on_demand=str(attrs.get("render", "on_demand")) != "always")
        self._camera_handles.append(handle)
        return handle

    def _author_fisheye(self, prim_path: str, spec: Dict[str, Any], width: int,
                        height: int) -> None:
        """Render through a real fisheye lens rather than a wide pinhole.

        A pinhole cannot express a field of 180 degrees or more - its focal length goes
        negative - so the arm-a camera was a 150 degree rectilinear stand-in for a 185
        degree lens. Isaac 6.1 dropped `fisheyeEquidistant`; what it has is the OpenCV
        fisheye (Kannala-Brandt) distortion schema, r = f * theta * (1 + k1 theta^2 + ...).
        With every k zero that is exactly an f-theta lens, r = f * theta, which is what the
        FUJINON FE185C057HA-1 is. `f` in pixels is chosen so the full field's image circle
        fits the frame's short side, unless the spec gives `focal_px`.
        """
        import math

        fov = float(spec.get("fov_deg", 185.0))
        focal = float(spec.get("focal_px") or
                      (min(width, height) / 2.0) / (math.radians(fov) / 2.0))
        stage = self._isaac.omni_usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        prim.ApplyAPI("OmniLensDistortionOpenCvFisheyeAPI")
        prim.GetAttribute("omni:lensdistortion:model").Set("opencvFisheye")
        base = "omni:lensdistortion:opencvFisheye:"
        for key, value in (("fx", focal), ("fy", focal), ("cx", width / 2.0),
                           ("cy", height / 2.0),
                           *((k, float(spec.get(k, 0.0))) for k in ("k1", "k2", "k3", "k4"))):
            attr = prim.GetAttribute(base + key)
            if attr:
                attr.Set(float(value))
        size = prim.GetAttribute(base + "imageSize")
        if size:
            from pxr import Gf
            size.Set(Gf.Vec2i(int(width), int(height)))
        LOGGER.info("camera %s: f-theta fisheye, %.0f deg, f %.1f px", prim_path, fov, focal)

    def _require_prim(self, prim_path: str) -> None:
        """Raise a helpful error if prim_path doesn't exist in the stage."""
        get_prim = getattr(self._isaac, "get_prim_at_path", None)
        if get_prim is None:
            return
        prim = get_prim(prim_path)
        if prim is None or not prim.IsValid():
            parent_path = prim_path.rsplit("/", 1)[0] or "/"
            hint = ""
            parent = get_prim(parent_path)
            if parent is not None and parent.IsValid():
                children = [c.GetName() for c in parent.GetChildren()]
                hint = f"; children of {parent_path}: {children}"
            raise ValueError(f"prim not found: {prim_path}{hint}")

    def create_base(self, name: str, attrs: Dict[str, Any]) -> "BaseHandle":
        self._require_booted()
        if self.mock:
            factory = lambda: MockBaseHandle(name, attrs)
        else:
            factory = lambda: self.run(
                lambda: self._create_base_isaac(name, attrs), timeout=120.0
            )
        return self._cached_handle("base", name, attrs, factory)

    def _create_base_isaac(self, name: str, attrs: Dict[str, Any]) -> "IsaacBaseHandle":
        from .spatial import to_vec3

        usd, meta = self._resolve_usd(attrs)
        prim_path = attrs.get("prim_path") or f"/World/{_prim_name(name)}"
        wheel_joints = attrs.get("wheel_joints") or meta.get("wheel_joints")
        if not wheel_joints or len(wheel_joints) != 2:
            raise ValueError(
                "base needs wheel_joints: [left_joint_name, right_joint_name] "
                "(known assets like 'jetbot' provide defaults)"
            )
        wheel_radius = float(attrs.get("wheel_radius", meta.get("wheel_radius", 0.05)))
        wheel_base = float(attrs.get("wheel_base", meta.get("wheel_base", 0.3)))
        position = to_vec3(attrs.get("position"))

        base_kwargs: Dict[str, Any] = dict(
            prim_path=prim_path,
            name=name,
            wheel_dof_names=list(wheel_joints),
            create_robot=usd is not None,
            usd_path=usd,
            position=list(position),
        )
        if attrs.get("orientation_wxyz") is not None:
            base_kwargs["orientation"] = [float(v) for v in attrs["orientation_wxyz"]]
        robot = self._isaac.WheeledRobot(**base_kwargs)
        self.world.scene.add(robot)
        self.world.reset()

        controller = self._isaac.DifferentialController(
            name=f"{name}_controller",
            wheel_radius=wheel_radius,
            wheel_base=wheel_base,
        )
        handle = IsaacBaseHandle(self, robot, controller, wheel_radius, wheel_base)
        self.world.add_physics_callback(f"{name}_drive", handle._on_physics_step)
        return handle


def _yaw_quat(yaw_deg: float):
    import numpy as np
    half = math.radians(yaw_deg) / 2.0
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)])


def _yaw_of(quat):
    """The yaw-only part of a quaternion, as a quaternion.

    A part resting on a deck has one meaningful rotation; keeping only that avoids
    re-staging a part with a tilt it picked up on the way.
    """
    import numpy as np
    w, x, y, z = [float(v) for v in quat]
    return _yaw_quat(math.degrees(math.atan2(2.0 * (w * z + x * y),
                                             1.0 - 2.0 * (y * y + z * z))))


def _quat_conj(q):
    import numpy as np
    return np.array([q[0], -q[1], -q[2], -q[3]])


def _quat_mul(a, b):
    import numpy as np
    aw, ax, ay, az = [float(v) for v in a]
    bw, bx, by, bz = [float(v) for v in b]
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def _quat_rotate(q, v):
    import numpy as np
    w, x, y, z = [float(t) for t in q]
    u = np.array([x, y, z])
    return (2.0 * float(np.dot(u, v)) * u
            + (w * w - float(np.dot(u, u))) * np.asarray(v)
            + 2.0 * w * np.cross(u, np.asarray(v)))


def _local_ip() -> str:
    """Best-effort primary local IP (no traffic is actually sent)."""
    import socket

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
        finally:
            s.close()
    except Exception:
        return ""


def _prim_name(name: str) -> str:
    """Component names may contain characters USD prim names can't."""
    return "".join(c if c.isalnum() or c == "_" else "_" for c in name)


def _import_isaac():
    """Import everything we need from isaac sim, tolerating the module
    renames across releases (isaacsim.* in >=4.5, omni.isaac.* before)."""

    class NS:
        pass

    ns = NS()

    try:
        from isaacsim.core.api import World
    except ImportError:
        from omni.isaac.core import World
    ns.World = World

    try:
        from isaacsim.core.utils.stage import add_reference_to_stage, open_stage
    except ImportError:
        from omni.isaac.core.utils.stage import add_reference_to_stage, open_stage
    ns.add_reference_to_stage = add_reference_to_stage
    ns.open_stage = open_stage

    try:
        from isaacsim.storage.native import get_assets_root_path
    except ImportError:
        from omni.isaac.core.utils.nucleus import get_assets_root_path
    ns.get_assets_root_path = get_assets_root_path

    try:
        from isaacsim.core.prims import SingleRigidPrim
        ns.SingleRigidPrim = SingleRigidPrim
    except ImportError:
        ns.SingleRigidPrim = None

    try:
        from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
    except ImportError:
        from omni.isaac.core.articulations import Articulation as SingleArticulation
        from omni.isaac.core.prims import XFormPrim as SingleXFormPrim
    ns.SingleArticulation = SingleArticulation
    ns.SingleXFormPrim = SingleXFormPrim

    try:
        from isaacsim.core.utils.types import ArticulationAction
    except ImportError:
        from omni.isaac.core.utils.types import ArticulationAction
    ns.ArticulationAction = ArticulationAction

    # Surface gripper: isaac's suction primitive. It fabricates a joint between the
    # gripper and whatever is within reach of its attachment points, which is exactly
    # what a vacuum tool does, and unlike a kinematic re-parent it can drop a part and
    # can fail to pick one up.
    try:
        import omni.usd

        from isaacsim.robot.surface_gripper import GripperView
        from usd.schema.isaac import robot_schema
        ns.GripperView = GripperView
        ns.robot_schema = robot_schema
        ns.omni_usd = omni.usd
        from . import compat as _compat
        ns.gripper_modules = _compat.import_surface_gripper()
    except ImportError:
        ns.GripperView = None
        ns.robot_schema = None
        ns.omni_usd = None
        ns.gripper_modules = None

    try:
        import omni.client
        ns.client = omni.client
    except ImportError:
        ns.client = None

    try:
        from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
    except ImportError:
        from omni.isaac.core.objects import DynamicCuboid, FixedCuboid
    ns.DynamicCuboid = DynamicCuboid
    ns.FixedCuboid = FixedCuboid

    try:
        from isaacsim.core.utils.prims import get_prim_at_path
    except ImportError:
        try:
            from omni.isaac.core.utils.prims import get_prim_at_path
        except ImportError:
            get_prim_at_path = None
    ns.get_prim_at_path = get_prim_at_path

    try:
        from isaacsim.sensors.camera import Camera
    except ImportError:
        from omni.isaac.sensor import Camera
    ns.Camera = Camera

    try:
        from isaacsim.robot.wheeled_robots.robots import WheeledRobot
        from isaacsim.robot.wheeled_robots.controllers.differential_controller import (
            DifferentialController,
        )
    except ImportError:
        from omni.isaac.wheeled_robots.robots import WheeledRobot
        from omni.isaac.wheeled_robots.controllers.differential_controller import (
            DifferentialController,
        )
    ns.WheeledRobot = WheeledRobot
    ns.DifferentialController = DifferentialController

    return ns


# ======================================================================
# Handles - the interface component models talk to. All public methods are
# safe to call from any thread.
# ======================================================================


class GripperHandle:
    """A suction gripper, as the module's components see it."""

    def close(self) -> None:
        raise NotImplementedError

    def open(self) -> None:
        raise NotImplementedError

    def status(self) -> str:
        """"Open", "Closing" or "Closed"."""
        raise NotImplementedError

    def gripped_objects(self) -> List[str]:
        raise NotImplementedError


class IsaacGripperHandle(GripperHandle):
    """Drives isaac's surface gripper.

    The suction is real physics, not a re-parent: isaac fabricates a joint to whatever
    body is within the attachment point's reach when the gripper closes. So a grab can
    fail because nothing was close enough, and a held part can be pulled off by its own
    weight or by driving it into something - which is the behaviour a vacuum cup has and
    a kinematic attach does not.
    """

    # The action value's sign is what matters, not its magnitude.
    _CLOSE = 0.5
    _OPEN = -0.5

    def __init__(self, sim: "SimManager", view: Any, prim_path: str,
                 jaw_prim: Optional[str] = None) -> None:
        self._sim = sim
        self._view = view
        self._prim_path = prim_path
        # Where the visible fingers hang, if this gripper was given any. The suction is
        # still what holds; the fingers are what makes the holding legible.
        self._jaw_prim = jaw_prim

    def close(self) -> None:
        self._sim.run(lambda: self._view.apply_gripper_action([self._CLOSE]))
        if self._jaw_prim:
            self._sim.set_jaw(self._jaw_prim, closed=True,
                              gap_mm=self._held_width_mm())

    def open(self) -> None:
        self._sim.run(lambda: self._view.apply_gripper_action([self._OPEN]))
        if self._jaw_prim:
            self._sim.set_jaw(self._jaw_prim, closed=False)

    def _held_width_mm(self) -> Optional[float]:
        """How wide the thing it just grabbed is, so the fingers close ONTO it.

        One `closed_mm` cannot serve two parts. This cell's jaw picks a 141 mm pin that is
        8 mm across and a flag that is 16.2 mm across, and whichever number is configured
        is wrong for the other one: at 10 mm the fingers sit 1 mm off the pin, which looks
        like a grip, and 6 mm INSIDE the flag, which looks like the finger is part of the
        flag. Neither reads as a gripper holding something.

        The gripper already knows what it grabbed, and gripping is by definition across a
        part's narrow axis, so the narrowest span of the held body's colliders is the
        opening to close to. Nothing held - a grab that found nothing - falls back to the
        configured `closed_mm`, which is also the honest thing to show.
        """
        try:
            held = self.gripped_objects()
        except Exception:  # noqa: BLE001 - a cosmetic width must never fail a grab
            return None
        if not held:
            return None
        try:
            got = self._sim.prim_bounds([held[0]]).get(held[0])
        except Exception:  # noqa: BLE001
            return None
        if not got:
            return None
        spans = [(got["max"][i] - got["min"][i]) * 1000.0 for i in range(3)]
        return min(spans) if min(spans) > 0.0 else None

    # isaac returns these as numbers, not the "Open"/"Closing"/"Closed" strings its
    # docstring advertises - measured: "0" open, "2" closed. Map both spellings so a
    # future build that starts returning words does not silently read as open.
    _STATUS = {"0": "Open", "1": "Closing", "2": "Closed",
               "Open": "Open", "Closing": "Closing", "Closed": "Closed"}

    def status(self) -> str:
        def _status():
            values = self._view.get_surface_gripper_status()
            if not len(values):
                return "Open"
            return self._STATUS.get(str(values[0]).strip(), str(values[0]))

        return self._sim.run(_status)

    def gripped_objects(self) -> List[str]:
        def _objects():
            if hasattr(self._view, "get_gripped_objects"):
                got = self._view.get_gripped_objects()
            else:   # Isaac 5.0's view lacks it; its interface takes one gripper's path
                path = self._view.prims[0].GetPath().pathString
                got = [self._view.surface_gripper_interface.get_gripped_objects(path)]
            if not len(got):
                return []
            first = got[0]
            # One gripper, so one entry - which is itself a list of prim paths.
            return [str(p) for p in (first if isinstance(first, (list, tuple)) else [first]) if p]

        return self._sim.run(_objects)


class MockGripperHandle(GripperHandle):
    """Tracks open/closed with no physics, so the module runs without isaac.

    It reports nothing gripped, deliberately. A mock that always claimed success would
    let a round pass in mock and fail the moment it met a simulator.
    """

    def __init__(self, name: str, attrs: Dict[str, Any]) -> None:
        self.name = name
        self._closed = False

    def close(self) -> None:
        self._closed = True

    def open(self) -> None:
        self._closed = False

    def status(self) -> str:
        return "Closed" if self._closed else "Open"

    def gripped_objects(self) -> List[str]:
        return []


class ArmHandle:
    def get_joint_positions(self) -> List[float]:  # radians
        raise NotImplementedError

    def set_joint_targets(self, positions: List[float]) -> None:
        raise NotImplementedError

    def is_moving(self) -> bool:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError

    def get_efforts(self) -> Dict[str, Any]:
        """What the drives are doing. Empty where a handle cannot say."""
        return {}

    def get_end_pose(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
        """((x,y,z) meters, (w,x,y,z) quaternion) of the end effector."""
        raise NotImplementedError


class IsaacArmHandle(ArmHandle):
    def __init__(self, sim: SimManager, articulation, ee_prim,
                 name: str = "") -> None:
        self._sim = sim
        self._art = articulation
        self._ee = ee_prim
        self._name = name

    def get_joint_positions(self) -> List[float]:
        return self._sim.run(lambda: [float(v) for v in self._art.get_joint_positions()])

    def set_joint_targets(self, positions: List[float]) -> None:
        import numpy as np

        def _apply():
            action = self._sim._isaac.ArticulationAction(
                joint_positions=np.array(positions, dtype=float)
            )
            self._art.apply_action(action)

        self._sim.run(_apply)

    def is_moving(self) -> bool:
        def _check():
            vels = self._art.get_joint_velocities()
            if vels is None:
                return False
            return bool(max(abs(float(v)) for v in vels) > 1e-2)

        return self._sim.run(_check)

    def get_efforts(self) -> Dict[str, Any]:
        """What the drives are actually doing, and what they are allowed to do.

        A position drive has no gravity compensation: it holds a pose by deflecting, so
        it settles where stiffness x error equals the load, and it cannot deflect further
        than max_effort allows. That makes "the arm stopped a degree short and kept
        creeping" a reading rather than a mystery - compare the measured effort against
        the limit, and the error against load/stiffness.
        """
        def _read():
            out: Dict[str, Any] = {}
            # The limit is read from BOTH the articulation and its controller, because
            # they disagree: set_max_efforts on the controller accepts a request for 300
            # and the readback still says the asset's authored 150. One of them is the
            # value physx enforces and one is the value the USD was authored with, and
            # which is which is not documented - so report both rather than pick, and
            # let a stall be judged against the pair.
            controller = getattr(self._art, "get_articulation_controller", None)
            controller = controller() if controller is not None else None
            for label, getter, target in (
                    ("measured", "get_measured_joint_efforts", self._art),
                    ("applied", "get_applied_joint_efforts", self._art),
                    ("limits", "get_max_efforts", self._art),
                    ("limits_from_controller", "get_max_efforts", controller)):
                fn = getattr(target, getter, None) if target is not None else None
                if fn is None and target is self._art and controller is not None:
                    fn = getattr(controller, getter, None)
                if fn is None:
                    continue
                try:
                    values = fn()
                except Exception as exc:  # noqa: BLE001
                    out[label] = f"unavailable: {exc}"
                    continue
                if values is not None:
                    out[label] = [round(float(v), 1) for v in values]
            try:
                gains = self._art.get_articulation_controller().get_gains()
                out["stiffness"] = [round(float(v), 1) for v in gains[0]]
                out["damping"] = [round(float(v), 1) for v in gains[1]]
            except Exception as exc:  # noqa: BLE001
                out["gains"] = f"unavailable: {exc}"
            out.update(self._sim._gain_notes.get(self._name, {}))
            out.update(self._sim._drive_force_notes.get(self._name, {}))
            return out

        return self._sim.run(_read)

    def stop(self) -> None:
        # hold the current position
        current = self.get_joint_positions()
        self.set_joint_targets(current)

    def get_end_pose(self):
        if self._ee is None:
            raise NotImplementedError(
                "set end_effector_prim in the arm config to report end position"
            )

        def _pose():
            pos, quat = self._ee.get_world_pose()
            return (
                (float(pos[0]), float(pos[1]), float(pos[2])),
                (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])),
            )

        return self._sim.run(_pose)


class MockArmHandle(ArmHandle):
    """Joints move linearly toward their targets at a fixed speed."""

    SPEED = 1.0  # rad/s per joint

    def __init__(self, name: str, attrs: Dict[str, Any]) -> None:
        self.name = name
        dof = int(attrs.get("mock_dof", 6))
        self._lock = threading.Lock()
        self._start = [0.0] * dof
        self._target = [0.0] * dof
        self._t0 = time.monotonic()

    def _positions_at(self, now: float) -> List[float]:
        out = []
        dt = max(0.0, now - self._t0)
        for s, t in zip(self._start, self._target):
            delta = t - s
            travel = self.SPEED * dt
            if abs(delta) <= travel:
                out.append(t)
            else:
                out.append(s + math.copysign(travel, delta))
        return out

    def get_joint_positions(self) -> List[float]:
        with self._lock:
            return self._positions_at(time.monotonic())

    def set_joint_targets(self, positions: List[float]) -> None:
        with self._lock:
            now = time.monotonic()
            self._start = self._positions_at(now)
            if len(positions) != len(self._start):
                raise ValueError(
                    f"expected {len(self._start)} joint positions, got {len(positions)}"
                )
            self._target = list(positions)
            self._t0 = now

    def is_moving(self) -> bool:
        with self._lock:
            pos = self._positions_at(time.monotonic())
            return any(abs(p - t) > 1e-9 for p, t in zip(pos, self._target))

    def stop(self) -> None:
        with self._lock:
            now = time.monotonic()
            self._start = self._positions_at(now)
            self._target = list(self._start)
            self._t0 = now

    def get_end_pose(self):
        # a fixed, deterministic pose for testing
        return ((0.3, 0.0, 0.3), (1.0, 0.0, 0.0, 0.0))


class CameraHandle:
    def set_rendering(self, on: Optional[bool] = None) -> Dict[str, Any]:
        """Turn the camera's rendering on or off; a camera that always renders says so."""
        return {"rendering": True, "switches": 0, "switches_all_cameras": 0}

    def get_rgb(self):
        """Return an (H, W, 3) uint8 numpy array."""
        raise NotImplementedError

    def get_rgbd(self):
        """Return ((H, W, 3) uint8, (H, W) float32 metres along the optical axis), from
        the same rendered frame. Only for a camera configured with `depth`."""
        raise NotImplementedError

    def labeled_frame(self, labels: Sequence[str]):
        """Return ((H, W, 3) uint8, [box]) from the same rendered frame: the tight 2D box
        of the visible pixels of every prim whose UsdSemantics `class` label is one of
        `labels`. A box is {"label", "x_min", "y_min", "x_max", "y_max"} in pixels."""
        return self.get_rgb(), []


class IsaacCameraHandle(CameraHandle):
    """An isaac camera, rendered ON DEMAND unless told to render always.

    Every camera's render product renders every frame whether anyone reads it or not, and
    this cell has nine: measured, the three tunnel cameras alone took the sim from 0.25
    to 0.17 of real time with the GPU at 100% (2026-10-12). An on-demand camera keeps its
    render product's updates off until it is turned on - by a read, or by the course's
    tools through `set_rendering` - and once on it renders every frame until it is turned
    OFF explicitly, by the course's tools.

    The renderer leaks when it goes from rendering NOTHING back to rendering something:
    205-215 MB a time, never given back (2026-10-17, L4; Isaac Sim 6.1.0 and 6.2.0 alike,
    and standalone with one plain camera and the viewport off, 111 MB). With any other
    render product live, the same switch costs nothing. So the renderer is never let go
    idle: SimManager keeps a 64x64 keepalive render product, on only while no camera is
    (`_render_handoff`), which costs ~20 ms a frame then and nothing once a camera is on.
    """

    FRESH_FRAMES = 3
    WAKE_TIMEOUT_S = 5.0
    switches = 0                         # on->off cycles this process, every camera

    def __init__(self, sim: SimManager, camera, on_demand: bool = False) -> None:
        self._sim = sim
        self._cam = camera
        self._on_demand = on_demand and self._texture() is not None
        self._lock = threading.Lock()
        self._live = True
        self._switches = 0
        self._boxes = None                 # the tight-box annotator, once labeled_frame asks
        if self._on_demand:
            self._set_live(False)

    def _texture(self):
        rp = getattr(self._cam, "_render_product", None)
        return getattr(rp, "hydra_texture", None)

    def _set_live(self, live: bool) -> None:
        tex = self._texture()
        self._sim.run(lambda: self._sim._render_handoff(self, tex, bool(live)))
        self._live = live

    def _sim_time(self) -> float:
        return self._sim.run(lambda: float(self._sim.world.current_time))

    def _wake(self) -> None:
        # Fresh frames are counted in SIM time: the camera's own `rendering_frame` is a
        # dict in this isaac, not a counter. One render per rendering_dt, so a few of
        # those after updates come back on, the frame read is one rendered since.
        self._set_live(True)
        self._wait_fresh()

    def set_rendering(self, on: Optional[bool] = None) -> Dict[str, Any]:
        """Turn this camera's rendering on or off (None: just report). Off-to-on waits
        for fresh frames, as a read does; on-to-off is the cycle that costs GPU memory."""
        if self._texture() is not None and on is not None:
            with self._lock:
                if on and not self._live:
                    self._wake()
                elif not on and self._live:
                    self._set_live(False)
                    self._switches += 1
                    IsaacCameraHandle.switches += 1
        return {"rendering": bool(self._live), "switches": self._switches,
                "switches_all_cameras": IsaacCameraHandle.switches}

    def get_rgb(self):
        if self._on_demand:
            with self._lock:
                if not self._live:
                    self._wake()
                return self._grab_now()
        return self._grab_now()

    def get_rgbd(self):
        def _grab():
            frame = self._cam.get_rgba()
            depth = self._cam.get_depth()
            if frame is None or frame.size == 0 or depth is None or depth.size == 0:
                raise RuntimeError("no depth frame yet - is this camera configured "
                                   "with `depth` and the simulation playing?")
            return frame[:, :, :3].copy(), depth.astype("float32").copy()

        if self._on_demand:
            with self._lock:
                if not self._live:
                    self._wake()
                return self._sim.run(_grab)
        return self._sim.run(_grab)

    def _wait_fresh(self) -> None:
        dt = self._sim.run(lambda: float(self._sim.world.get_rendering_dt()))
        start = self._sim_time()
        deadline = time.monotonic() + self.WAKE_TIMEOUT_S
        while time.monotonic() < deadline:
            if self._sim_time() >= start + (self.FRESH_FRAMES + 1) * dt:
                return
            time.sleep(0.02)
        LOGGER.warning("camera %s: no fresh frame %.0f s after waking it",
                       getattr(self._cam, "prim_path", "?"), self.WAKE_TIMEOUT_S)

    def labeled_frame(self, labels: Sequence[str]):
        # The renderer's tight-box annotator on this camera's own render product, attached
        # the first time it is asked for: the boxes are of the pixels it rendered, so a
        # decal on the far side of the shell, or behind an arm, has no box. Read in the
        # same sim-thread call as the colour, so both are of one render.
        want = {str(lab) for lab in labels}

        def _attach():
            import omni.replicator.core as rep

            ann = rep.AnnotatorRegistry.get_annotator(
                "bounding_box_2d_tight", init_params={"semanticTypes": ["class"]})
            ann.attach([self._cam._render_product_path])
            return ann

        def _grab():
            frame = self._cam.get_rgba()
            got = self._boxes.get_data()
            if frame is None or frame.size == 0:
                raise RuntimeError("no frame available yet - is the simulation playing?")
            names = (got.get("info") or {}).get("idToLabels") or {}
            boxes = []
            for row in got.get("data", []):
                entry = names.get(str(int(row["semanticId"]))) or names.get(int(row["semanticId"]))
                label = entry.get("class", "") if isinstance(entry, dict) else str(entry or "")
                for one in label.split(","):
                    if one.strip() in want:
                        boxes.append({"label": one.strip(), "x_min": int(row["x_min"]),
                                      "y_min": int(row["y_min"]), "x_max": int(row["x_max"]),
                                      "y_max": int(row["y_max"])})
            return frame[:, :, :3].copy(), boxes

        with self._lock:
            fresh = self._boxes is None
            if fresh:
                self._boxes = self._sim.run(_attach)
            if self._on_demand and not self._live:
                self._wake()
            elif fresh:
                self._wait_fresh()
            return self._sim.run(_grab)

    def _grab_now(self):
        def _grab():
            frame = self._cam.get_rgba()
            if frame is None or frame.size == 0:
                raise RuntimeError(
                    "no frame available yet - is the simulation playing?"
                )
            return frame[:, :, :3].copy()

        return self._sim.run(_grab)


class MockCameraHandle(CameraHandle):
    def __init__(self, name: str, attrs: Dict[str, Any]) -> None:
        self.name = name
        self._w = int(attrs.get("width", 640))
        self._h = int(attrs.get("height", 480))

    def get_rgb(self):
        import numpy as np

        # gradient background with a time-based moving bar so images change
        img = np.zeros((self._h, self._w, 3), dtype=np.uint8)
        img[:, :, 0] = np.linspace(0, 255, self._w, dtype=np.uint8)[None, :]
        img[:, :, 1] = np.linspace(0, 255, self._h, dtype=np.uint8)[:, None]
        x = int((time.monotonic() * 60) % self._w)
        img[:, max(0, x - 5) : x + 5, :] = 255
        return img


class BaseHandle:
    def set_velocity(self, linear_mps: float, angular_rps: float) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError

    def is_moving(self) -> bool:
        raise NotImplementedError


class IsaacBaseHandle(BaseHandle):
    def __init__(self, sim: SimManager, robot, controller, wheel_radius: float, wheel_base: float) -> None:
        self._sim = sim
        self._robot = robot
        self._controller = controller
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self._cmd = (0.0, 0.0)
        self._lock = threading.Lock()

    def _on_physics_step(self, step_size: float) -> None:
        # runs on the sim thread every physics step
        with self._lock:
            lin, ang = self._cmd
        try:
            self._robot.apply_wheel_actions(self._controller.forward(command=[lin, ang]))
        except Exception:
            LOGGER.exception("error driving base")

    def set_velocity(self, linear_mps: float, angular_rps: float) -> None:
        with self._lock:
            self._cmd = (float(linear_mps), float(angular_rps))

    def stop(self) -> None:
        self.set_velocity(0.0, 0.0)

    def is_moving(self) -> bool:
        with self._lock:
            return self._cmd != (0.0, 0.0)


class MockBaseHandle(BaseHandle):
    def __init__(self, name: str, attrs: Dict[str, Any]) -> None:
        self.name = name
        self.wheel_radius = float(attrs.get("wheel_radius", 0.05))
        self.wheel_base = float(attrs.get("wheel_base", 0.3))
        self._cmd = (0.0, 0.0)
        self._lock = threading.Lock()

    def set_velocity(self, linear_mps: float, angular_rps: float) -> None:
        with self._lock:
            self._cmd = (float(linear_mps), float(angular_rps))

    def stop(self) -> None:
        self.set_velocity(0.0, 0.0)

    def is_moving(self) -> bool:
        with self._lock:
            return self._cmd != (0.0, 0.0)
