import logging
import queue

import bpy

from urmoco.scheduler import Scheduler
from urmoco.config import Config
from urmoco.blender.constants import ARMATURE_MODEL
from urmoco.blender.operators.base_modal_operator import \
    get_synced_modal_operator_class
from urmoco.blender.rig import get_q
from urmoco.blender.state import Mode, get_mode, set_mode, set_status_text

logger = logging.getLogger(__name__)


def get_operators(config: Config, scheduler: Scheduler):
    base_operator = get_synced_modal_operator_class(
        config, scheduler
    )

    class CalibrationOperator(base_operator):
        bl_idname = "urmoco.calibration"
        bl_label = "Calibrate robot"

        @classmethod
        def poll(cls, context):
            return get_mode() is Mode.ON

        def on_execute(self, context):
            # Clear DFMoco queues before entering capturing mode to get rid of queued up and potentially old commands
            while not scheduler.df_in_q.empty():
                try:
                    scheduler.df_in_q.get_nowait()
                except queue.Empty:
                    pass
            while not scheduler.df_out_q.empty():
                try:
                    scheduler.df_out_q.get_nowait()
                except queue.Empty:
                    pass

            scheduler.ur_in_q.put({"type": "calibration"})
            set_mode(Mode.CALIBRATION)
            set_status_text("Started calibration")

        def on_request(self, context, request):
            if request["type"] == "calibration_success":
                set_mode(Mode.ON)
                set_status_text("Calibration success")

            if request["type"] == "calibration_failure":
                set_mode(Mode.ON)
                set_status_text("Calibration failed")

    return [CalibrationOperator]
