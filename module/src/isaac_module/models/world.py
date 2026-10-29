"""viam-education:isaac-sim:world - the generic component that owns the simulator.

Configure exactly one of these per machine. All other isaac-sim components
name it in their "world" attribute; their validate_config returns it as an
implicit dependency so viam-server boots the world first.

Attributes:
  mock (bool, default false)        - run without isaac sim (for dev/testing)
  headless (bool, default true)     - run kit without a local GUI window
  livestream (bool, default true)   - enable WebRTC livestreaming (view with
                                      the Isaac Sim WebRTC Streaming Client)
  livestream_public_ip (string)     - IP advertised to streaming clients;
                                      auto-detected if unset
  livestream_width / _height (int)  - streamed resolution, default 1280x720
  usd_stage (string)                - USD file/omniverse URL to open; if unset
                                      an empty stage with a ground plane is used
  physics_dt / rendering_dt (float) - sim step sizes, default 1/60
  max_rtf (float) - the fastest the sim may run against the wall clock; 0 = unpaced
  boot_timeout_sec (float)          - how long to wait for kit to boot
  kit_log_level (string)            - kit console verbosity, default "warning"
DoCommand "prop_poses" returns where each prop actually is now - {position_mm,
orientation_wxyz, live} per name, optionally filtered by a "names" list. Config says where
a prop was spawned; this says where it is, which is what lets a vision service be scored
against ground truth rather than against itself.

DoCommand "obstacles" returns every cube prop as {label, fixed, center_mm, dims_mm},
ready to become viam Geometry obstacles. The motion service cannot see props on its own -
they live in isaac, not in the frame system - so callers must pass these in `world_state`
on every Move or the arm will plan straight through the scenery.

Attributes:
  ground (object)                   - the floor: {"kind": "plane"|"grid"|"none",
                                      "size", "color", "friction", "restitution",
                                      "matte"}. The default grid reads as a CAD
                                      viewport; a plane reads as a room.
  render (object)                   - {"viewport_grid": bool}
  join / unjoin                     - fasten one body to another as the assembly would:
                                      {"name", "body0", "body1", "kind": fixed|revolute,
                                       "anchor_mm" (WORLD mm) OR "local0_mm"/"local1_mm"
                                       (each body's own frame - published frames pull the
                                       part square, a world anchor freezes in whatever the
                                       grasp got wrong), "axis", "limits_deg",
                                       "drive": {"stiffness", "damping", "target_deg",
                                       "max_force"} for a sprung hinge}.
                                      Parts do not stay assembled because they are in the
                                      right place; a seated part needs the joint its
                                      asset says holds it.
  lighting.lamps (list)             - named lights: {"name", "kind": sphere|disk,
                                      "intensity", "radius_m", "color",
                                      "parent_prim", "local_position",
                                      "local_orientation_rpy_deg"}. With a parent_prim
                                      the lamp rides that prim - an inspection flash on
                                      an arm's flange, beside its camera.
  lighting (object)                 - {"dome": {"intensity", "color", "texture",
                                      "texture_format", "rotation_deg"},
                                      "sphere_intensity": float}
  props (list)                      - objects spawned into the scene at boot:
                                      {"name", "type": "cube"|"usd",
                                       "position": [x,y,z] meters,
                                       "size" (m), "scale" [sx,sy,sz],
                                       "color" [r,g,b] 0-1, "fixed" (bool),
                                       "usd_path" (for type usd)}

DoCommand:
  {"command": "status"} | {"command": "play"} | {"command": "pause"} |
  {"command": "reset"} |
  {"command": "add_usd", "usd_path": "...", "prim_path": "/World/thing",
   "position": [x, y, z]}
"""

from typing import Any, ClassVar, Dict, Mapping, Optional, Sequence, Tuple

from typing_extensions import Self
from viam.components.generic import Generic
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes, struct_to_dict

from .. import FAMILY, NAMESPACE
from ..sim_manager import SimConfig, SimManager


class IsaacWorld(Generic, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily(NAMESPACE, FAMILY), "world")

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        w = cls(config.name)
        w.reconfigure(config, dependencies)
        return w

    @classmethod
    def validate_config(
        cls, config: ComponentConfig
    ) -> Tuple[Sequence[str], Sequence[str]]:
        attrs = struct_to_dict(config.attributes)
        for key in (
            "physics_dt",
            "rendering_dt",
            "boot_timeout_sec",
            "livestream_width",
            "livestream_height",
        ):
            if key in attrs and float(attrs[key]) <= 0:
                raise ValueError(f"{key} must be positive")
        return [], []

    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> None:
        attrs = struct_to_dict(config.attributes)
        cfg = SimConfig(
            mock=bool(attrs.get("mock", False)),
            headless=bool(attrs.get("headless", True)),
            livestream=bool(attrs.get("livestream", True)),
            usd_stage=attrs.get("usd_stage") or None,
            physics_dt=float(attrs.get("physics_dt", 1.0 / 60.0)),
            rendering_dt=float(attrs.get("rendering_dt", 1.0 / 60.0)),
            max_rtf=float(attrs.get("max_rtf", 0.0)),
            boot_timeout=float(attrs.get("boot_timeout_sec", 300.0)),
            kit_log_level=str(attrs.get("kit_log_level", "warning")),
            livestream_public_ip=str(attrs.get("livestream_public_ip", "")),
            livestream_width=int(attrs.get("livestream_width", 1280)),
            livestream_height=int(attrs.get("livestream_height", 720)),
            props=[dict(p) for p in attrs.get("props", [])],
            ground=dict(attrs["ground"]) if attrs.get("ground") else None,
            render=dict(attrs.get("render") or {}),
            lighting=dict(attrs.get("lighting") or {}),
        )
        SimManager.get().ensure_booted(cfg)

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Mapping[str, ValueTypes]:
        sim = SimManager.get()
        cmd = str(command.get("command", ""))
        if cmd == "status":
            return sim.status()
        if cmd == "play":
            sim.play()
            return {"ok": True}
        if cmd == "pause":
            sim.pause()
            return {"ok": True}
        if cmd == "reset":
            sim.reset()
            return {"ok": True}
        if cmd == "obstacles":
            return {"obstacles": sim.prop_obstacles()}
        if cmd == "prop_poses":
            names = command.get("names")
            names = [str(n) for n in names] if names else None
            return {"props": sim.prop_poses(names)}
        if cmd == "defects":
            return {"defects": sim.defects(str(command.get("prop", "part")))}
        if cmd == "show_defects":
            if "visible" not in command:
                raise ValueError("show_defects requires 'visible'")
            return sim.show_defects(
                str(command.get("prop", "part")), bool(command["visible"]),
                command.get("render_steps"))
        if cmd == "set_defects":
            wanted = command.get("defects")
            if not isinstance(wanted, dict):
                raise ValueError("set_defects requires a 'defects' map of name -> bool")
            return {"defects": sim.set_defects(
                str(command.get("prop", "part")),
                {str(k): bool(v) for k, v in wanted.items()},
                render_steps=command.get("render_steps"))}
        if cmd == "variants":
            return {"variants": sim.variants(str(command.get("prop", "part")))}
        if cmd == "set_light":
            return {"light": sim.set_light(str(command["path"]), float(command["intensity"]))}
        if cmd == "set_variants":
            wanted = command.get("variants")
            if not isinstance(wanted, dict):
                raise ValueError("set_variants requires a 'variants' map")
            return {"variants": sim.set_variants(
                str(command.get("prop", "part")),
                {str(k): str(v) for k, v in wanted.items()},
                render_steps=command.get("render_steps"))}
        if cmd == "prop_attributes":
            names = [str(x) for x in (command.get("names") or [])]
            if not names:
                raise ValueError("prop_attributes requires a 'names' list")
            return {"attributes": sim.prop_attributes(
                str(command.get("prop", "part")), names)}
        if cmd == "prim_poses":
            paths = command.get("paths")
            if not isinstance(paths, list):
                raise ValueError("prim_poses requires a 'paths' list")
            return {"prims": sim.prim_poses([str(p) for p in paths])}
        if cmd == "set_door":
            return {"door": sim.set_door(
                str(command.get("prop", "part")),
                float(command.get("degrees", 0.0)),
                str(command.get("body", "Door")),
                str(command.get("axis", "x")))}
        if cmd == "join":
            body0, body1 = command.get("body0"), command.get("body1")
            if not body0 or not body1:
                raise ValueError("join requires 'body0' and 'body1' prim paths")
            return {"join": sim.join(
                str(command.get("name", "seat")), str(body0), str(body1),
                kind=str(command.get("kind", "fixed")),
                anchor_mm=command.get("anchor_mm"),
                axis=command.get("axis"),
                limits_deg=command.get("limits_deg"),
                local0_mm=command.get("local0_mm"),
                local1_mm=command.get("local1_mm"),
                drive=command.get("drive"),
                release_kinematic=bool(command.get("release_kinematic", False)))}
        if cmd == "unjoin":
            return {"unjoin": sim.unjoin(str(command.get("name", "seat")))}
        if cmd == "place_props":
            items = command.get("props")
            if not isinstance(items, list):
                raise ValueError("place_props requires a 'props' list")
            return {"placed": sim.place_props([dict(p) for p in items])}
        if cmd == "set_translate":
            return {"set": sim.set_translate(
                str(command.get("path")),
                [float(v) for v in (command.get("translate_mm") or (0.0, 0.0, 0.0))])}
        if cmd == "place_prop":
            return {"placed": sim.place_prop(
                str(command.get("name", "part")),
                position_mm=command.get("position_mm"),
                yaw_deg=(None if command.get("yaw_deg") is None
                         else float(command["yaw_deg"])),
                orientation_wxyz=command.get("orientation_wxyz"))}
        if cmd == "conveyor":
            name = str(command.get("name", "belt"))
            run = command.get("run")
            return {"conveyor": sim.conveyor(
                name,
                run=None if run is None else bool(run),
                speed_mm_s=(None if command.get("speed_mm_s") is None
                            else float(command["speed_mm_s"])))}
        if cmd == "index":
            return {"index": sim.index(str(command.get("action", "read")),
                                       dict(command.get("spec") or {}))}
        if cmd == "raycast":
            origin = command.get("from_mm")
            along = command.get("along")
            if not isinstance(origin, list) or not isinstance(along, list):
                raise ValueError("raycast requires 'from_mm' and 'along' lists")
            return {"ray": sim.raycast(
                [float(v) for v in origin], [float(v) for v in along],
                float(command.get("distance_mm", 200.0)))}
        if cmd == "prim_bounds":
            paths = command.get("paths")
            if not isinstance(paths, list):
                raise ValueError("prim_bounds requires a 'paths' list")
            return {"bounds": sim.prim_bounds([str(p) for p in paths])}
        if cmd == "prim_attrs":
            paths = command.get("paths")
            if not isinstance(paths, list):
                raise ValueError("prim_attrs requires a 'paths' list")
            return {"attrs": sim.prim_attrs([str(p) for p in paths])}
        if cmd == "prim_masses":
            paths = [str(x) for x in (command.get("paths") or [])]
            if not paths:
                raise ValueError("prim_masses requires a 'paths' list")
            return {"masses": sim.prim_masses(paths)}
        if cmd == "prim_poses":
            paths = [str(x) for x in (command.get("paths") or [])]
            if not paths:
                raise ValueError("prim_poses requires a 'paths' list")
            return {"prims": sim.prim_poses(paths)}
        if cmd == "set_kinematic":
            return {"set": sim.set_kinematic(
                str(command.get("name", "")), bool(command.get("on", True)),
                position_mm=command.get("position_mm"),
                orientation_wxyz=command.get("orientation_wxyz"))}
        if cmd == "profile":
            return {"profile": sim.profile(int(command.get("steps", 60)),
                                           hide=command.get("hide"),
                                           render_every=command.get("render_every"))}
        if cmd == "toggle_probe":
            return {"toggle_probe": sim.toggle_probe(str(command["camera"]),
                                                     int(command.get("toggles", 20)),
                                                     int(command.get("every", 5)))}
        if cmd == "kit_settings":
            return {"settings": sim.kit_settings([str(k) for k in command.get("keys", [])],
                                                 values=command.get("set"))}
        if cmd == "leak_probe":
            return {"leak_probe": sim.leak_probe(str(command["camera"]),
                                                 command.get("mechanisms"),
                                                 int(command.get("cycles", 6)),
                                                 settings=command.get("settings"),
                                                 hide=command.get("hide"),
                                                 keepalive=int(command.get("keepalive", 0)))}
        if cmd == "render_probe":
            return {"render_probe": sim.render_probe(command.get("cameras"),
                                                     int(command.get("frames", 4)))}
        if cmd == "audit":
            return {"audit": sim.audit(
                str(command.get("action", "read")),
                targets=[dict(t) for t in (command.get("targets") or [])],
                every_n=int(command.get("every_n", 4)),
                shrink=float(command.get("shrink", 1.0)))}
        if cmd == "add_usd":
            usd_path = str(command.get("usd_path", ""))
            prim_path = str(command.get("prim_path", ""))
            if not usd_path or not prim_path:
                raise ValueError("add_usd requires usd_path and prim_path")
            position = command.get("position") or [0.0, 0.0, 0.0]
            sim.add_usd_reference(
                usd_path, prim_path, tuple(float(v) for v in position)
            )
            return {"ok": True}
        raise ValueError(
            f"unknown command {cmd!r}; supported: status, play, pause, reset, conveyor, place_prop, place_props, set_door, "
            "defects, set_defects, variants, set_variants, prop_attributes, "
            f"obstacles, prop_poses, prim_poses, prim_bounds, prim_attrs, prim_masses, "
            f"raycast, "
            f"join, unjoin, add_usd"
        )
