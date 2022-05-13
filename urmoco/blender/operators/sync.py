import logging

from urmoco.blender.state import Mode, get_mode
from urmoco.blender.operators.base_modal_operator import get_synced_modal_operator_class


logger = logging.getLogger(__name__)


def get_operators(config, urmoco_in_queue, urmoco_out_queue):
    base_operator = get_synced_modal_operator_class(config, urmoco_in_queue, urmoco_out_queue)

    class SyncPoseOperator(base_operator):
        bl_idname = "urmoco.sync"
        bl_label = "Sync"

        @classmethod
        def poll(cls, context):
            return get_mode(context) is Mode.ON

        def on_request(self, context, request):
            if request["type"] == "sync":
                return {'FINISHED'}

    return [SyncPoseOperator]
