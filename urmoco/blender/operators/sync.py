import logging

from urmoco.blender.constants import (ARMATURE_GHOST, ARMATURE_MODEL,
                                      BONE_IK_CONTROL)
from urmoco.blender.operators.base_modal_operator import \
    get_synced_modal_operator_class
from urmoco.blender.rig import apply_q, get_q, has_constraints
from urmoco.blender.state import Mode, get_mode, set_status_text

logger = logging.getLogger(__name__)


def get_operators(config, urmoco_in_queue, urmoco_out_queue):
    base_operator = get_synced_modal_operator_class(
        config, urmoco_in_queue, urmoco_out_queue
    )

    class SyncPoseOperator(base_operator):
        bl_idname = "urmoco.sync"
        bl_label = "Sync"

        @classmethod
        def poll(cls, context):
            return get_mode(context) is Mode.ON and not has_constraints(
                ARMATURE_MODEL, BONE_IK_CONTROL
            )

        def on_request(self, context, request):
            if request["type"] == "sync":
                new_configuration = get_q(ARMATURE_GHOST)
                apply_q(ARMATURE_MODEL, new_configuration)
                set_status_text(context, "Synced pose from physical robot")
                return {"FINISHED"}

    return [SyncPoseOperator]
