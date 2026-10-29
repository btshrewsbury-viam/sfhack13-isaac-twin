"""viam-education:isaac-sim:camera - a simulated RGB camera.

Attributes:
  world (string, required)        - name of the viam-education:isaac-sim:world component
  prim_path (string)              - existing camera prim to attach to, or
                                    where to create one (default /World/<name>)
  width / height (int)            - resolution, default 640x480
  position ([x,y,z] meters)       - where to place a newly created camera
  target ([x,y,z] meters)         - aim the camera at this point (easiest way
                                    to make a scene-monitor camera)
  orientation_rpy_deg ([r,p,y])   - explicit orientation instead of target
  fov_deg (float)                 - horizontal field of view; newly created
                                    cameras default to 70 (the usd default
                                    lens is a ~24 degree telephoto)
  parent_prim (string)            - create the camera as a child of this prim
                                    so it moves with it, e.g. an arm's wrist
                                    link for an end-effector camera
  local_position ([x,y,z] m)      - offset from parent_prim, default [0,0,0.05]
  local_orientation_rpy_deg       - orientation relative to parent_prim;
                                    default [180,0,0] = look out the +Z
                                    (tool) axis
  depth (bool)                    - also render depth; get_point_cloud then
                                    returns the scene in the camera's frame
  pcd_stride (int)                - keep every Nth pixel in each direction for
                                    the point cloud, default 2
  pcd_max_m (float)               - drop points farther than this, default 5
  render ("on_demand" | "always") - on_demand (default): not rendered until read or
                                    turned on, then rendered until turned off

DoCommand:
  {"command": "set_rendering", "on": true|false}   turn rendering on or off; without
      "on", report. Returns {"rendering", "switches", "switches_all_cameras"}. Switching
      costs no GPU memory: the module never lets the renderer go idle, which is what
      leaked (see IsaacCameraHandle).
  {"command": "prim"}   the camera's prim path in the stage: {"prim_path"}.
  {"command": "labeled_frame", "labels": [class, ...]}   a frame and the tight 2D boxes
      of the prims carrying those UsdSemantics `class` labels (defect_smudge, defect_drip,
      defect_overspray, mailbox_body ...), both from one render: {"image" (base64 JPEG),
      "width", "height", "boxes": [{"label", "x_min", "y_min", "x_max", "y_max",
      "x_min_normalized", "y_min_normalized", "x_max_normalized", "y_max_normalized"}]}.
      Only visible pixels are boxed. The training captures and paint-oracle both read it.
"""

import asyncio
import base64
import math
from io import BytesIO
from typing import Any, ClassVar, Dict, List, Mapping, Optional, Sequence, Tuple

from typing_extensions import Self
from viam.components.camera import Camera
from viam.media.video import CameraMimeType, NamedImage, ViamImage
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResponseMetadata, ResourceName
from viam.proto.component.camera import GetPropertiesResponse, IntrinsicParameters
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily

from .. import FAMILY, NAMESPACE
from ..sim_manager import CameraHandle, SimManager
from .utils import apply_frame_to_attrs, get_attrs, validate_sim_component


class IsaacCamera(Camera, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily(NAMESPACE, FAMILY), "camera")

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._handle: Optional[CameraHandle] = None
        self._attrs: Dict[str, Any] = {}

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        cam = cls(config.name)
        cam.reconfigure(config, dependencies)
        return cam

    @classmethod
    def validate_config(
        cls, config: ComponentConfig
    ) -> Tuple[Sequence[str], Sequence[str]]:
        # cameras can always be created fresh, no asset/usd required
        return validate_sim_component(config, needs_source=False)

    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> None:
        attrs = get_attrs(config)
        # A camera riding a link (parent_prim) is placed by its local mount; its `frame`
        # (parent: the arm) only tells viam where it is, for transform_pose. Folded into
        # the spawn attrs it would be read as a WORLD position and put the camera there.
        if not attrs.get("parent_prim"):
            attrs = apply_frame_to_attrs(config, attrs)
        self._attrs = attrs
        self._handle = SimManager.get().create_camera(self.name, attrs)

    def _h(self) -> CameraHandle:
        if self._handle is None:
            raise RuntimeError(f"camera {self.name} is not attached to the sim")
        return self._handle

    def _encode(self, mime_type: str = "", arr=None) -> ViamImage:
        from PIL import Image

        arr = self._h().get_rgb() if arr is None else arr
        img = Image.fromarray(arr, mode="RGB")
        buf = BytesIO()
        if mime_type == CameraMimeType.PNG:
            img.save(buf, format="PNG")
            return ViamImage(buf.getvalue(), CameraMimeType.PNG)
        img.save(buf, format="JPEG", quality=90)
        return ViamImage(buf.getvalue(), CameraMimeType.JPEG)

    async def do_command(self, command: Mapping[str, Any], *, timeout: Optional[float] = None,
                         **kwargs) -> Mapping[str, Any]:
        cmd = command.get("command")
        if cmd == "set_rendering":
            on = command.get("on")
            on = None if on is None else bool(on)
            return await asyncio.to_thread(self._h().set_rendering, on)
        if cmd == "prim":
            # Where the camera is in the stage, so a caller can read its pose from the
            # world (viam-education:isaac-sim:oracle projects the cell's truth through it).
            return {"prim_path": str(self._h()._cam.prim_path)}
        if cmd == "labeled_frame":
            return await asyncio.to_thread(self._labeled_frame, list(command.get("labels") or []))
        raise NotImplementedError(f"camera {self.name}: unknown command {cmd!r}")

    def _labeled_frame(self, labels: List[str]) -> Dict[str, Any]:
        arr, boxes = self._h().labeled_frame(labels)
        h, w = int(arr.shape[0]), int(arr.shape[1])
        for b in boxes:
            b.update({"x_min_normalized": b["x_min"] / w, "y_min_normalized": b["y_min"] / h,
                      "x_max_normalized": b["x_max"] / w, "y_max_normalized": b["y_max"] / h})
        image = self._encode(CameraMimeType.JPEG, arr)
        return {"image": base64.b64encode(image.data).decode("ascii"), "width": w,
                "height": h, "boxes": boxes}

    async def get_image(self, mime_type: str = "", **kwargs) -> ViamImage:
        return await asyncio.to_thread(self._encode, mime_type)

    async def get_images(
        self, **kwargs
    ) -> Tuple[List[NamedImage], ResponseMetadata]:
        image = await asyncio.to_thread(self._encode, CameraMimeType.JPEG)
        # A depth camera names its colour image "color", as the RealSense module does:
        # viam:vision:detections-to-segments looks for exactly that source name and
        # refuses the camera without it ("did not return a 'color' image").
        name = "color" if self._attrs.get("depth") else self.name
        named = NamedImage(name=name, data=image.data, mime_type=image.mime_type)
        return [named], ResponseMetadata()

    async def get_point_cloud(self, **kwargs) -> Tuple[bytes, str]:
        if not self._attrs.get("depth"):
            raise NotImplementedError(f"camera {self.name} has no depth: set `depth`")
        return await asyncio.to_thread(self._pcd), CameraMimeType.PCD

    def _pcd(self) -> bytes:
        """The depth frame as a binary PCD: x y z in METRES in the camera's own frame
        (+Z along the optical axis, +X image right, +Y image down - the frame viam gives
        this camera), back-projected through the same intrinsics `get_properties`
        reports, with each point's colour."""
        import numpy as np

        rgb, depth = self._h().get_rgbd()
        k = self.intrinsics()
        stride = max(1, int(self._attrs.get("pcd_stride", 2)))
        far = float(self._attrs.get("pcd_max_m", 5.0))
        h, w = depth.shape
        v, u = np.mgrid[0:h:stride, 0:w:stride]
        z = depth[::stride, ::stride]
        c = rgb[::stride, ::stride].reshape(-1, 3).astype(np.uint32)
        ok = (np.isfinite(z) & (z > 0.0) & (z < far)).reshape(-1)
        z = z.reshape(-1)[ok]
        # Pixel centres: the renderer samples at +0.5.
        x = ((u.reshape(-1)[ok] + 0.5) - k.center_x_px) * z / k.focal_x_px
        y = ((v.reshape(-1)[ok] + 0.5) - k.center_y_px) * z / k.focal_y_px
        packed = (c[ok, 0] << 16) | (c[ok, 1] << 8) | c[ok, 2]
        pts = np.zeros(len(z), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                      ("rgb", "<u4")])
        pts["x"], pts["y"], pts["z"], pts["rgb"] = x, y, z, packed
        head = (f"VERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\n"
                f"COUNT 1 1 1 1\nWIDTH {len(z)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
                f"POINTS {len(z)}\nDATA binary\n")
        return head.encode() + pts.tobytes()

    async def get_geometries(self, **kwargs):
        # cameras occupy no space; needed so the motion service can build a
        # world state when this camera has a frame
        return []

    async def get_properties(self, **kwargs) -> GetPropertiesResponse:
        # Intrinsics only, no distortion_parameters, even for the fisheye: viam-server's
        # camera client parses that field with transform.NewDistorter, which knows only the
        # Brown-Conrady models, and any other model fails get_properties for every caller.
        # The fisheye's model (OpenCV fisheye, k1..k4 = 0) lives in its config attributes.
        return GetPropertiesResponse(supports_pcd=bool(self._attrs.get("depth")),
                                     intrinsic_parameters=self.intrinsics())

    def intrinsics(self) -> Optional[IntrinsicParameters]:
        """The intrinsics the camera renders with: square pixels, centred. A pinhole from
        the horizontal field the module sets on the lens; for the fisheye, its f-theta
        focal (its model is in the config attributes). None for a camera whose field the
        config does not state."""
        a = self._attrs
        w, h = int(a.get("width", 640)), int(a.get("height", 480))
        if a.get("fisheye"):
            # f-theta: the same focal the module renders with (sim_manager's
            # _apply_fisheye): the shorter side's half-width over half the field, in radians.
            spec = dict(a["fisheye"])
            f = float(spec.get("focal_px") or (min(w, h) / 2.0)
                      / (math.radians(float(spec.get("fov_deg", 185.0))) / 2.0))
            return IntrinsicParameters(width_px=w, height_px=h, focal_x_px=f, focal_y_px=f,
                                       center_x_px=w / 2.0, center_y_px=h / 2.0)
        if a.get("prim_path") and not a.get("fov_deg"):
            return None
        f = (w / 2.0) / math.tan(math.radians(float(a.get("fov_deg", 70.0))) / 2.0)
        return IntrinsicParameters(width_px=w, height_px=h, focal_x_px=f, focal_y_px=f,
                                   center_x_px=w / 2.0, center_y_px=h / 2.0)
