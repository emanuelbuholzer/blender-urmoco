import logging

from urmoco.blender.operators.base_modal_operator import get_synced_modal_operator_class
from urmoco.blender.state import Mode, set_mode, set_status_text, get_mode

logger = logging.getLogger(__name__)


def get_operators(config, urmoco_in_queue, urmoco_out_queue):
    base_operator = get_synced_modal_operator_class(config, urmoco_in_queue, urmoco_out_queue)

    class StopOperator(base_operator):
        bl_idname = "urmoco.emergency_stop"
        bl_label = "Stop"

        @classmethod
        def poll(cls, context):
            return get_mode(context) is not Mode.OFF

        def on_execute(self, context):
            urmoco_in_queue.put({"type": "stop"})
            set_mode(context, Mode.AWAIT_RESPONSE)
            set_status_text(context, "Stopping")

        def on_request(self, context, request):
            if request["type"] == "stop":
                set_mode(context, Mode.ON)
                set_status_text(context, "Robot stopped")
                return {'FINISHED'}

    return [StopOperator]
