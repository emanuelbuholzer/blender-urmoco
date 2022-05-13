import bpy
import logging

from urmoco.blender.state import Mode, set_mode, set_status_text, get_mode

logger = logging.getLogger(__name__)


def get_operators(config, urmoco_in_queue, urmoco_out_queue):

    class StopOperator(bpy.types.Operator):
        bl_idname = "urmoco.emergency_stop"
        bl_label = "Stop"

        @classmethod
        def poll(cls, context):
            return get_mode(context) is not Mode.OFF

        def execute(self, context):
            urmoco_in_queue.put({"type": "stop"})
            set_mode(context, Mode.AWAIT_RESPONSE)
            set_status_text(context, "Stopping")
            return {'FINISHED'}

    return [StopOperator]
