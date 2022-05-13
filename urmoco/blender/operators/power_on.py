import logging

from urmoco.blender.state import Mode, set_mode, set_status_text, get_mode
from urmoco.blender.operators.base_modal_operator import get_synced_modal_operator_class


logger = logging.getLogger(__name__)


def get_operators(config, urmoco_in_queue, urmoco_out_queue):
    base_operator = get_synced_modal_operator_class(config, urmoco_in_queue, urmoco_out_queue)

    class PowerOnOperator(base_operator):
        bl_idname = "urmoco.power_on"
        bl_label = "Startup"

        @classmethod
        def poll(cls, context):
            return get_mode(context) is Mode.OFF

        def on_execute(self, context):
            urmoco_in_queue.put({"type": "power_on"})
            set_mode(context, Mode.AWAIT_RESPONSE)
            set_status_text(context, "Powering on")

        def on_request(self, context, request):
            if request["type"] == "sync":
                return {'FINISHED'}

    return [PowerOnOperator]
