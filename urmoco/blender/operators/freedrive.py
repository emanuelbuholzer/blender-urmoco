import logging

import bpy

from urmoco.scheduler import Scheduler
from urmoco.config import Config
from urmoco.blender.operators.base_modal_operator import \
    get_synced_modal_operator_class
from urmoco.blender.state import Mode, get_mode, set_mode, set_status_text

logger = logging.getLogger(__name__)


def get_operators(config: Config, scheduler: Scheduler):
    base_operator = get_synced_modal_operator_class(
        config, scheduler
    )

    class StartFreedriveOperator(base_operator):
        bl_idname = "urmoco.start_freedrive"
        bl_label = "Start Freedrive"

        @classmethod
        def poll(cls, context):
            return get_mode() is Mode.ON

        def on_execute(self, context):
            scheduler.ur_in_q.put({"type": "start_freedrive"})
            set_mode(Mode.FREEDRIVE)
            set_status_text("Freedrive started")

    class StopFreedriveOperator(bpy.types.Operator):
        bl_idname = "urmoco.stop_freedrive"
        bl_label = "Stop Freedrive"

        @classmethod
        def poll(cls, context):
            return get_mode() is Mode.FREEDRIVE

        def execute(self, context):
            scheduler.ur_in_q.put({"type": "stop_freedrive"})
            context.window_manager.urmoco_state.running_in_modal = False
            set_mode(Mode.ON)
            set_status_text("Freedrive stopped")
            return {"FINISHED"}

    return [StartFreedriveOperator, StopFreedriveOperator]
