import logging

from urmoco.blender.operators.base_modal_operator import get_synced_modal_operator_class
from urmoco.blender.state import Mode, get_mode, set_mode, set_status_text
from urmoco.config import Config
from urmoco.scheduler import Scheduler

logger = logging.getLogger(__name__)


def get_operators(config: Config, scheduler: Scheduler):
    base_operator = get_synced_modal_operator_class(config, scheduler)

    class StopOperator(base_operator):
        bl_idname = "urmoco.emergency_stop"
        bl_label = "Stop"

        @classmethod
        def poll(cls, context):
            return get_mode() is not Mode.OFF

        def on_execute(self, context):
            scheduler.ur_in_q.put({"type": "stop"})
            set_mode(Mode.AWAIT_RESPONSE)
            set_status_text("Stopping")

    return [StopOperator]
