import logging

from urmoco.blender.constants import ARMATURE_MODEL
from urmoco.blender.state import Mode, set_mode, set_status_text, get_mode
from urmoco.blender.operators.base_modal_operator import get_synced_modal_operator_class
from urmoco.blender.rig import get_q


logger = logging.getLogger(__name__)


def get_operators(config, urmoco_in_queue, urmoco_out_queue):
    base_operator = get_synced_modal_operator_class(config, urmoco_in_queue, urmoco_out_queue)

    class TransferPoseOperator(base_operator):
        bl_idname = "urmoco.transfer"
        bl_label = "Transfer"

        @classmethod
        def poll(cls, context):
            return get_mode(context) is Mode.ON

        def on_execute(self, context):
            configuration = get_q(ARMATURE_MODEL)

            urmoco_in_queue.put({
                "type": "transfer",
                "payload": {
                    "target_joints": configuration
                }
            })
            set_mode(context, Mode.AWAIT_RESPONSE)
            set_status_text(context, "Moving to target")

        def on_request(self, context, request):
            if request["type"] == "stop":
                set_mode(context, Mode.ON)
                set_status_text(context, "Stopped before target")
                return {'CANCELLED'}

            if request["type"] == "move_success":
                set_mode(context, Mode.ON)
                set_status_text(context, "Stopped at target")
                return {'FINISHED'}

            if request["type"] == "move_timeout":
                set_mode(context, Mode.ON)
                set_status_text(context, f"Move timed out (not at target)")
                return {'CANCELLED'}

    return [TransferPoseOperator]
