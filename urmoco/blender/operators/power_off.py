import logging

import bpy

from urmoco.blender.operators.base_modal_operator import get_synced_modal_operator_class
from urmoco.blender.rig import set_ghost_hidden
from urmoco.blender.state import Mode, get_mode, set_mode, set_status_text
from urmoco.config import Config
from urmoco.scheduler import Scheduler

logger = logging.getLogger(__name__)


def get_operators(config: Config, scheduler: Scheduler):
    base_operator = get_synced_modal_operator_class(config, scheduler)

    class PowerOffOperator(base_operator):
        bl_idname = "urmoco.power_off"
        bl_label = "Power off"

        @classmethod
        def poll(cls, context):
            return get_mode() in {Mode.ON, Mode.LOCKED}

        def on_execute(self, context):
            scheduler.ur_in_q.put({"type": "power_off"})
            set_mode(Mode.AWAIT_RESPONSE)
            set_status_text("Powering off")

    return [PowerOffOperator]
