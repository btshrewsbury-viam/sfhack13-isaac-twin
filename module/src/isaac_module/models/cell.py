"""viam-education:isaac-sim:cell - the learner-facing cell service (Viam 103, plan R1).

Every verb is one DoCommand, `{"<verb>": {<args>}}`, with the request and reply shapes of
the viam-103 learner contract section 2; `cell_rules.CellRules` holds them. This model is
the Viam face of those rules and the place the physical cell's motion plugs in.

Attributes:
  motion (bool, default false) - drive the sim cell for each verb (the cycle's own stations,
                                 probes/assembly/cell_run.py). Off, the verbs move nothing:
                                 the mock cell, or a machine without the assembly cell.
  assets_dir (string)          - the usd tree the station code reads its frames from: the
                                 cell's asset package on a learner's machine
                                 ("${packages.archive.cell-assets}/usd"); default this
                                 workstation's ~/viam/usd
  world, arms, grips, camera   - the cell's resources when `motion` is on; default
                                 "sim-world", ["arm-a", "arm-b", "arm-c"],
                                 {"arm-b": "grip-b", "arm-c": "grip-c"}, "grasp-cam"
"""

import os
import sys
from pathlib import Path
from typing import Any, ClassVar, Dict, Mapping, Optional, Sequence, Tuple

from typing_extensions import Self
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.services.generic import Generic
from viam.utils import ValueTypes, struct_to_dict

from .. import FAMILY, NAMESPACE
from ..cell_rules import CELLS, CellRules


class IsaacCell(Generic, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily(NAMESPACE, FAMILY), "cell")

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        cell = cls(config.name)
        cell.reconfigure(config, dependencies)
        return cell

    @staticmethod
    def _names(attrs: Dict[str, Any]) -> Dict[str, Any]:
        return {"world": str(attrs.get("world", "sim-world")),
                "arms": [str(a) for a in attrs.get("arms", ["arm-a", "arm-b", "arm-c"])],
                "grips": {str(k): str(v) for k, v in
                          (attrs.get("grips") or {"arm-b": "grip-b", "arm-c": "grip-c"}).items()},
                "camera": str(attrs.get("camera", "grasp-cam"))}

    @classmethod
    def validate_config(
        cls, config: ComponentConfig
    ) -> Tuple[Sequence[str], Sequence[str]]:
        attrs = struct_to_dict(config.attributes)
        if not attrs.get("motion"):
            return [], []
        n = cls._names(attrs)
        return [n["world"], *n["arms"], *n["grips"].values(), n["camera"]], []

    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> None:
        attrs = struct_to_dict(config.attributes)
        # A reconfigure keeps the batch: the learner's score must survive a config save.
        self.rules = CELLS.get(self.name) or CellRules()
        CELLS[self.name] = self.rules
        if attrs.get("motion"):
            by_name = {rn.name: dep for rn, dep in dependencies.items()}
            n = self._names(attrs)
            if attrs.get("assets_dir"):
                # Before the station code is imported: it reads the frames at import.
                os.environ["CELL_USD_DIR"] = str(attrs["assets_dir"])
            self.rules.motion = _sim_motion(
                by_name[n["world"]], {a: by_name[a] for a in n["arms"]},
                {a: by_name[g] for a, g in n["grips"].items()}, by_name[n["camera"]])

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs: Any,
    ) -> Mapping[str, ValueTypes]:
        return await self.rules.do_command(dict(command))

    async def close(self) -> None:
        CELLS.pop(self.name, None)


def _sim_motion(world, arms, grips, camera):
    """The cell's motion over the assembly cell's own station code (probes/assembly), which
    ships beside src/ in the module archive."""
    from viam.services.motion import MotionClient

    root = Path(__file__).resolve().parents[3] / "probes"
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    from assembly.cell_run import CellMotion

    # The builtin motion service is viam-server's own; its client rides the same channel
    # the dependencies came over.
    return CellMotion(world, arms, grips, MotionClient("builtin", world.channel), camera)
