import logging

import bpy

from urmoco.blender.operators.base_modal_operator import \
    get_synced_modal_operator_class
from urmoco.blender.rig import set_ghost_hidden
from urmoco.blender.state import Mode, get_mode, set_mode, set_status_text

logger = logging.getLogger(__name__)


def get_operators(config, scheduler, urmoco_in_queue, urmoco_out_queue):
    base_operator = get_synced_modal_operator_class(
        config, urmoco_in_queue, urmoco_out_queue
    )

    class PowerOffOperator(base_operator):
        bl_idname = "urmoco.power_off"
        bl_label = "Power off"

        @classmethod
        def poll(cls, context):
            return get_mode(context) in {Mode.ON, Mode.LOCKED}

        def on_execute(self, context):
            urmoco_in_queue.put({"type": "power_off"})
            set_mode(context, Mode.AWAIT_RESPONSE)
            set_status_text(context, "Powering off")

        def on_request(self, context, request):
            if request["type"] == "power_off":
                set_mode(context, Mode.UNINITIALIZED)
                set_status_text(context, "Powered urmoco off")
                set_ghost_hidden(True)
                scheduler.terminate_backend()
                scheduler.terminate_dfmoco_server()
                return {"FINISHED"}

    return [PowerOffOperator]
