import logging

import bpy

from urmoco.blender.operators.base_modal_operator import \
    get_synced_modal_operator_class
from urmoco.blender.state import Mode, get_mode, set_mode, set_status_text

logger = logging.getLogger(__name__)


def get_operators(config, urmoco_in_queue, urmoco_out_queue):
    base_operator = get_synced_modal_operator_class(
        config, urmoco_in_queue, urmoco_out_queue
    )

    class StartFreedriveOperator(base_operator):
        bl_idname = "urmoco.start_freedrive"
        bl_label = "Start Freedrive"

        @classmethod
        def poll(cls, context):
            return get_mode(context) is Mode.ON

        def on_execute(self, context):
            urmoco_in_queue.put({"type": "start_freedrive"})
            set_mode(context, Mode.FREEDRIVE)
            set_status_text(context, "Freedrive started")

    class StopFreedriveOperator(bpy.types.Operator):
        bl_idname = "urmoco.stop_freedrive"
        bl_label = "Stop Freedrive"

        @classmethod
        def poll(cls, context):
            return get_mode(context) is Mode.FREEDRIVE

        def execute(self, context):
            urmoco_in_queue.put({"type": "stop_freedrive"})
            context.window_manager.urmoco_state.running_in_modal = False
            set_mode(context, Mode.ON)
            set_status_text(context, "Freedrive stopped")
            return {"FINISHED"}

    return [StartFreedriveOperator, StopFreedriveOperator]
