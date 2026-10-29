"""viam-education:isaac-sim:scoreboard - the batch's counters and score, as sensor readings.

Reads the `cell` service it names (attribute `cell`, default "cell"): the keys of the
viam-103 learner contract section 2, flat snake_case, every number a float.

Attributes:
  cell (string, default "cell") - the name of the viam-education:isaac-sim:cell service to report
"""

from typing import Any, ClassVar, Mapping, Optional, Sequence, Tuple

from typing_extensions import Self
from viam.components.sensor import Sensor
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import SensorReading, struct_to_dict

from .. import FAMILY, NAMESPACE
from ..cell_rules import CELLS


class IsaacScoreboard(Sensor, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily(NAMESPACE, FAMILY), "scoreboard")

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        board = cls(config.name)
        board.reconfigure(config, dependencies)
        return board

    @classmethod
    def validate_config(
        cls, config: ComponentConfig
    ) -> Tuple[Sequence[str], Sequence[str]]:
        # The cell is a dependency so viam-server builds it first.
        return [str(struct_to_dict(config.attributes).get("cell", "cell"))], []

    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> None:
        self.cell_name = str(struct_to_dict(config.attributes).get("cell", "cell"))

    async def get_readings(
        self,
        *,
        extra: Optional[Mapping[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs: Any,
    ) -> Mapping[str, SensorReading]:
        rules = CELLS.get(self.cell_name)
        if rules is None:
            raise ValueError(f"no viam-education:isaac-sim:cell named {self.cell_name!r} in this module")
        return rules.readings()
