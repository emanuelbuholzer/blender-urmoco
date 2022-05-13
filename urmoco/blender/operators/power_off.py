import bpy
import logging

from urmoco.blender.state import Mode, set_mode, set_status_text, get_mode

logger = logging.getLogger(__name__)


def get_operators(config, urmoco_in_queue, urmoco_out_queue):

    class PowerOffOperator(bpy.types.Operator):
        bl_idname = "urmoco.power_off"
        bl_label = "Power off"

        @classmethod
        def poll(cls, context):
            return get_mode(context) is not Mode.OFF

        def execute(self, context):
            urmoco_in_queue.put({"type": "power_off"})
            set_mode(context, Mode.OFF)
            set_status_text(context, "Powered off")
            return {'FINISHED'}

    return [PowerOffOperator]
