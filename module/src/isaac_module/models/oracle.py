"""viam-education:isaac-sim:oracle - a vision service that answers from the cell's truth (viam-103 R1).

Page 10's `--vision oracle` runs the line with these in place of the learner's vision
services, so its report is what a perfect model would score. Each answers what the course
fake (`exercises/offline_cell.py`, FakeVision with `oracle`) answers, but from the sim:
the truth of the shell at the station from the `cell` service, and every pixel projected
from where the parts and cameras really are.

Attributes:
  kind (string, required) - infeed | inside | features | paint | shell
  cell (string, default "cell")        - the viam-education:isaac-sim:cell to read the truth from
  world (string, default "sim-world")  - the viam-education:isaac-sim:world to read poses from
  cameras (list)          - the cameras this oracle projects into; default by kind:
                            features realsense-cam, paint the three tunnel cameras,
                            shell grasp-cam

  infeed, inside   classifications: the truth's label at confidence 1, the others at 0
  features         detections on realsense-cam at the pin or flag stop: `bore` or
                   `flag_hole`, a box round the feature's true entry centre
  paint            detections on a tunnel camera at the paint check: the defect's label,
                   the renderer's tight box of its decal's visible pixels (the camera's
                   labeled_frame), on each camera that sees it
  shell            object point clouds on grasp-cam at the reject stop: grasp-cam's own
                   cloud inside the shell's projected footprint, boxed as rdk's
                   detections-to-segments boxes it (centre the points' mean, dims their
                   extents, millimetres in the camera's frame)
"""

import math
import struct
from typing import Any, ClassVar, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np
from typing_extensions import Self
from viam.media.video import ViamImage
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import (GeometriesInFrame, Geometry, PointCloudObject, Pose,
                               RectangularPrism, ResourceName, Vector3)
from viam.proto.service.vision import Classification, Detection
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.services.vision import CaptureAllResult, Vision
from viam.utils import ValueTypes, struct_to_dict

from .. import FAMILY, NAMESPACE
from ..cell_rules import CELLS

KINDS = ("infeed", "inside", "features", "paint", "shell")
LABELS = {"infeed": ("good", "damaged", "empty"), "inside": ("clean", "defect")}
DEFAULT_CAMERAS = {"features": ["realsense-cam"],
                   "paint": ["tunnel-top", "tunnel-left", "tunnel-right"],
                   "shell": ["grasp-cam"]}
# The features' entry centres are the shell asset's own published attributes (metres,
# shell frame), read from the stage rather than copied here.
FEATURE = {"pin": ("bore", "assembly:pinBore:entry"),
           "flag": ("flag_hole", "assembly:flagHole:entry")}
FEATURE_HALF_MM = 20.0         # 40 mm about the feature, plus 3 px: pilot B (Felix, 2026-10-19)
# Each defect's UsdSemantics class on the shell's decal (mailbox_usd).
DECALS = {"smudge": "defect_smudge", "drip": "defect_drip", "overspray": "defect_overspray"}
# A roof decal seen edge-on from a side camera is a real box 3 to 11 px tall that no model
# could learn (ray table, 2026-10-18), nor one under 16 px once the detector scales the
# frame to 320-384 px; the training captures drop the same (capture_paint.py).
MIN_BOX_PX = 16


class IsaacOracle(Vision, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily(NAMESPACE, FAMILY), "oracle")

    @classmethod
    def new(cls, config: ComponentConfig,
            dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        oracle = cls(config.name)
        oracle.reconfigure(config, dependencies)
        return oracle

    @staticmethod
    def _attrs(config) -> Dict[str, Any]:
        a = struct_to_dict(config.attributes)
        kind = str(a.get("kind", ""))
        if kind not in KINDS:
            raise ValueError(f"oracle `kind` must be one of {KINDS}, got {kind!r}")
        return {"kind": kind, "cell": str(a.get("cell", "cell")),
                "world": str(a.get("world", "sim-world")),
                "cameras": [str(c) for c in a.get("cameras", DEFAULT_CAMERAS.get(kind, []))]}

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Tuple[Sequence[str], Sequence[str]]:
        a = cls._attrs(config)
        deps = [a["cell"]] + ([a["world"], *a["cameras"]] if a["cameras"] else [])
        return deps, []

    def reconfigure(self, config: ComponentConfig,
                    dependencies: Mapping[ResourceName, ResourceBase]) -> None:
        a = self._attrs(config)
        self.kind, self.cell_name = a["kind"], a["cell"]
        by_name = {rn.name: dep for rn, dep in dependencies.items()}
        self.world = by_name.get(a["world"])
        self.cameras = {c: by_name[c] for c in a["cameras"] if c in by_name}
        self._prims: Dict[str, str] = {}

    # -- the truth and the geometry

    def _rules(self):
        rules = CELLS.get(self.cell_name)
        if rules is None:
            raise ValueError(f"no viam-education:isaac-sim:cell named {self.cell_name!r}")
        return rules

    async def _poses(self, paths: List[str]) -> Dict[str, Any]:
        return (await self.world.do_command({"command": "prim_poses", "paths": paths}))["prims"]

    async def _view(self, camera_name: str):
        """The camera's optical frame in the world (rotation columns: image right, image
        down, along the view; origin in mm) and its intrinsics."""
        cam = self.cameras[camera_name]
        if camera_name not in self._prims:
            self._prims[camera_name] = (await cam.do_command({"command": "prim"}))["prim_path"]
        path = self._prims[camera_name]
        got = (await self._poses([path]))[path]
        ux, uy, uz = (np.array(got[k], float) for k in ("x_axis", "y_axis", "z_axis"))
        rot = np.stack([ux, -uy, -uz], axis=1)
        k = (await cam.get_properties()).intrinsic_parameters
        return rot, np.array(got["position_mm"], float), k

    @staticmethod
    def _project(rot, at, k, world_mm) -> Tuple[float, float, float]:
        c = rot.T @ (np.asarray(world_mm, float) - at)
        return (k.focal_x_px * c[0] / c[2] + k.center_x_px,
                k.focal_y_px * c[1] / c[2] + k.center_y_px, float(c[2]))

    @staticmethod
    def _box(label: str, u0, v0, u1, v1, k) -> Detection:
        w, h = k.width_px, k.height_px
        return Detection(x_min=int(max(0, math.floor(u0))), y_min=int(max(0, math.floor(v0))),
                         x_max=int(min(w - 1, math.ceil(u1))), y_max=int(min(h - 1, math.ceil(v1))),
                         confidence=1.0, class_name=label,
                         x_min_normalized=max(0.0, u0 / w), y_min_normalized=max(0.0, v0 / h),
                         x_max_normalized=min(1.0, u1 / w), y_max_normalized=min(1.0, v1 / h))

    # -- classifications

    def _classify(self, count: int) -> List[Classification]:
        if self.kind not in LABELS:
            raise ValueError(f"the {self.kind} oracle is not a classifier")
        shell = self._rules().current()
        if shell is None:
            return []
        truth = shell[self.kind]
        out = [Classification(class_name=truth, confidence=1.0)]
        out += [Classification(class_name=lab, confidence=0.0)
                for lab in LABELS[self.kind] if lab != truth]
        return out[:int(count)]

    async def get_classifications_from_camera(self, camera_name: str, count: int, *,
                                              extra=None, timeout=None, **kw):
        return self._classify(count)

    async def get_classifications(self, image: ViamImage, count: int, *, extra=None,
                                  timeout=None, **kw):
        return self._classify(count)

    # -- detections

    async def get_detections_from_camera(self, camera_name: str, *, extra=None,
                                         timeout=None, **kw) -> List[Detection]:
        rules = self._rules()
        shell, station = rules.current(), rules.s.station
        if shell is None or camera_name not in self.cameras or shell["infeed"] == "empty":
            return []
        if self.kind == "features":
            return await self._feature(camera_name, station)
        if self.kind == "paint":
            return await self._paint(camera_name, shell, station)
        raise ValueError(f"the {self.kind} oracle is not a detector")

    async def get_detections(self, image: ViamImage, *, extra=None, timeout=None, **kw):
        raise NotImplementedError("the oracle answers from a camera: use "
                                  "get_detections_from_camera")

    async def _feature(self, camera_name: str, station: str) -> List[Detection]:
        if station not in FEATURE:
            return []
        label, attr = FEATURE[station]
        local = (await self.world.do_command({"command": "prop_attributes", "prop": "shell",
                                              "names": [attr]}))["attributes"][attr]
        sh = (await self._poses(["/World/shell"]))["/World/shell"]
        ax = [np.array(sh[k], float) for k in ("x_axis", "y_axis", "z_axis")]
        entry = np.array(sh["position_mm"], float) + sum(ax[i] * float(local[i]) * 1000.0
                                                         for i in range(3))
        rot, at, k = await self._view(camera_name)
        u, v, z = self._project(rot, at, k, entry)
        if z <= 0:
            return []
        half = FEATURE_HALF_MM * k.focal_x_px / z + 3.0
        return [self._box(label, u - half, v - half, u + half, v + half, k)]

    async def _paint(self, camera_name: str, shell, station: str) -> List[Detection]:
        defect = shell["paint"]
        if station != "paint" or defect not in DECALS:
            return []
        # The camera's own labeled_frame: the renderer's tight box of the decal's visible
        # pixels, the same call the training captures read, so the model is trained and
        # scored on one box source. A decal out of view, or on the far side, has no box.
        got = await self.cameras[camera_name].do_command(
            {"command": "labeled_frame", "labels": [DECALS[defect]]})
        w, h = float(got["width"]), float(got["height"])
        return [Detection(x_min=int(b["x_min"]), y_min=int(b["y_min"]),
                          x_max=int(b["x_max"]), y_max=int(b["y_max"]),
                          confidence=1.0, class_name=defect,
                          x_min_normalized=b["x_min"] / w, y_min_normalized=b["y_min"] / h,
                          x_max_normalized=b["x_max"] / w, y_max_normalized=b["y_max"] / h)
                for b in got.get("boxes", [])
                if min(b["x_max"] - b["x_min"], b["y_max"] - b["y_min"]) + 1 >= MIN_BOX_PX]

    # -- object point clouds

    async def get_object_point_clouds(self, camera_name: str, *, extra=None, timeout=None,
                                      **kw) -> List[PointCloudObject]:
        if self.kind != "shell":
            raise ValueError(f"the {self.kind} oracle does not segment")
        rules = self._rules()
        shell = rules.current()
        if shell is None or rules.s.station != "grasp" or camera_name not in self.cameras:
            return []
        cam = self.cameras[camera_name]
        blob, _mime = await cam.get_point_cloud()
        pts = _parse_pcd(blob)                                   # metres, camera frame
        rot, at, k = await self._view(camera_name)
        got = await self.world.do_command({"command": "prim_bounds", "paths": ["/World/shell"]})
        b = (got.get("bounds") or got)["/World/shell"]
        lo, hi = np.array(b["min"], float) * 1000.0, np.array(b["max"], float) * 1000.0
        corners = [[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1])
                   for z in (lo[2], hi[2])]
        uv = np.array([self._project(rot, at, k, c)[:2] for c in corners])
        (u0, v0), (u1, v1) = uv.min(0), uv.max(0)
        z = np.where(pts[:, 2] > 0, pts[:, 2], np.nan)
        u = k.focal_x_px * pts[:, 0] / z + k.center_x_px
        v = k.focal_y_px * pts[:, 1] / z + k.center_y_px
        inside = (u >= u0) & (u < u1) & (v >= v0) & (v < v1)
        sel = pts[inside]
        if not len(sel):
            return []
        mm = sel * 1000.0
        c, d = mm.mean(0), mm.max(0) - mm.min(0)
        geom = Geometry(center=Pose(x=float(c[0]), y=float(c[1]), z=float(c[2]),
                                    o_x=0, o_y=0, o_z=1, theta=0), label="shell",
                        box=RectangularPrism(dims_mm=Vector3(x=float(d[0]), y=float(d[1]),
                                                             z=float(d[2]))))
        return [PointCloudObject(point_cloud=_pcd_bytes(sel),
                                 geometries=GeometriesInFrame(reference_frame=camera_name,
                                                              geometries=[geom]))]

    # -- the rest of the API

    async def capture_all_from_camera(self, camera_name: str, return_image: bool = False,
                                      return_classifications: bool = False,
                                      return_detections: bool = False,
                                      return_object_point_clouds: bool = False, *,
                                      extra=None, timeout=None, **kw) -> CaptureAllResult:
        out = CaptureAllResult()
        if return_image and camera_name in self.cameras:
            named = (await self.cameras[camera_name].get_images())[0][0]
            out.image = ViamImage(named.data, named.mime_type)
        if return_classifications and self.kind in LABELS:
            out.classifications = self._classify(len(LABELS[self.kind]))
        if return_detections and self.kind in ("features", "paint"):
            out.detections = await self.get_detections_from_camera(camera_name)
        if return_object_point_clouds and self.kind == "shell":
            out.objects = await self.get_object_point_clouds(camera_name)
        return out

    async def get_properties(self, *, extra=None, timeout=None, **kw) -> Vision.Properties:
        return Vision.Properties(classifications_supported=self.kind in LABELS,
                                 detections_supported=self.kind in ("features", "paint"),
                                 object_point_clouds_supported=self.kind == "shell")

    async def do_command(self, command: Mapping[str, ValueTypes], *, timeout=None,
                         **kw) -> Mapping[str, ValueTypes]:
        raise NotImplementedError(f"oracle {self.name}: no commands")


def _parse_pcd(blob: bytes) -> np.ndarray:
    head, _, body = blob.partition(b"DATA binary\n")
    fields = sizes = None
    n = 0
    for line in head.decode().splitlines():
        key, *rest = line.split()
        if key == "FIELDS":
            fields = rest
        elif key == "SIZE":
            sizes = [int(v) for v in rest]
        elif key == "POINTS":
            n = int(rest[0])
    step = sum(sizes)
    raw = np.frombuffer(body[: n * step], dtype=np.uint8).reshape(n, step)
    cols, off = [], 0
    for f, sz in zip(fields, sizes):
        if f in ("x", "y", "z"):
            cols.append(raw[:, off:off + 4].copy().view("<f4").reshape(n))
        off += sz
    return np.stack(cols, axis=1).astype(np.float64)


def _pcd_bytes(xyz_m: np.ndarray, rgb: int = 0x9AA0AA) -> bytes:
    """x y z rgb, as grasp-cam's own cloud and rdk's detections-to-segments write it: a
    reader that takes the rows as four 4-byte fields (the course's grasp_point does) gets
    the points it expects."""
    n = len(xyz_m)
    head = (f"VERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\n"
            f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n").encode()
    rows = np.zeros(n, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")])
    rows["x"], rows["y"], rows["z"] = (np.asarray(xyz_m[:, i], "<f4") for i in range(3))
    rows["rgb"] = rgb
    return head + rows.tobytes()
