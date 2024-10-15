import logging

from urmoco.scheduler import Scheduler
from urmoco.config import Config
from urmoco.blender.constants import ARMATURE_MODEL
from urmoco.blender.operators.base_modal_operator import \
    get_synced_modal_operator_class
from urmoco.blender.rig import get_q
from urmoco.blender.state import Mode, get_mode, set_mode, set_status_text
import numpy as np

logger = logging.getLogger(__name__)


def get_operators(config: Config, scheduler: Scheduler):
    base_operator = get_synced_modal_operator_class(
        config, scheduler
    )

    class TransferPoseOperator(base_operator):
        bl_idname = "urmoco.transfer"
        bl_label = "Transfer"

        @classmethod
        def poll(cls, context):
            return get_mode() is Mode.ON

        def on_execute(self, context):
            configuration = get_q(ARMATURE_MODEL)

            scheduler.ur_in_q.put(
                {"type": "transfer", "payload": {"target_joints": configuration}}
            )
            set_mode(Mode.MOVING)
            set_status_text("Moving to target")

        def on_request(self, context, request):
            if request["type"] == "stop":
                set_mode(Mode.ON)
                set_status_text("Stopped before target")
                # scheduler.ur_in_q.put({"type": "sync"})
                return {"CANCELLED"}

            if request["type"] == "move_success":
                set_mode(Mode.ON)
                set_status_text("Stopped at target")
                # scheduler.ur_in_q.put({"type": "sync"})
                return {"FINISHED"}

            if request["type"] == "move_timeout":
                set_mode(Mode.ON)
                set_status_text(f"Move timed out (not at target)")
                # scheduler.ur_in_q.put({"type": "sync"})
                return {"CANCELLED"}

            if request["type"] == "move_cancelled":
                set_mode(Mode.ON)
                set_status_text(f"Move cancelled")
                scheduler.ur_in_q.put({"type": "sync"})
                return {"CANCELLED"}

    return [TransferPoseOperator]
