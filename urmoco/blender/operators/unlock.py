import logging

from urmoco.scheduler import Scheduler
from urmoco.config import Config
from urmoco.blender.operators.base_modal_operator import \
    get_synced_modal_operator_class
from urmoco.blender.state import Mode, get_mode, set_mode, set_status_text

logger = logging.getLogger(__name__)


def get_operators(config: Config, scheduler: Scheduler):
    base_operator = get_synced_modal_operator_class(
        config, scheduler
    )

    class UnlockOperator(base_operator):
        bl_idname = "urmoco.unlock"
        bl_label = "Unlock"

        @classmethod
        def poll(cls, context):
            return get_mode(context) is Mode.LOCKED

        def on_execute(self, context):
            scheduler.ur_in_q.put({"type": "unlock"})
            set_mode(context, Mode.AWAIT_RESPONSE)
            set_status_text(context, "Unlocking")

        def on_request(self, context, request):
            if request["type"] == "unlock":
                set_mode(context, Mode.ON)
                set_status_text(context, "Unlocked robot")
                return {"FINISHED"}

    return [UnlockOperator]
