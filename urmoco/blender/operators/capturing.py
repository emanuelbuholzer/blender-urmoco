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

    class StartCapturingOperator(base_operator):
        bl_idname = "urmoco.start_capturing"
        bl_label = "Start capturing"

        @classmethod
        def poll(cls, context):
            return get_mode() is Mode.ON

        _last_frame = None

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

            scheduler.ur_in_q.put({"type": "start_capturing"})
            set_mode(Mode.SHOOTING)
            set_status_text("Started capturing")

        def on_request(self, context, request):
            if request["type"] == "stop":
                set_mode(Mode.SHOOTING)
                set_status_text("Stopped move before frame")

            if request["type"] == "move_to_frame":
                frame = int(request["payload"]["target_frame"])
                self._last_frame = frame

                bpy.context.scene.frame_set(frame)
                configuration = get_q(ARMATURE_MODEL)
                scheduler.ur_in_q.put(
                    {
                        "type": "transfer",
                        "payload": {
                            "target_joints": configuration,
                            "target_frame": frame,
                        },
                    }
                )
                set_status_text(f"Moving to frame {frame}")

            if request["type"] == "move_success":
                set_status_text(f"Moved to frame {self._last_frame}")

            if request["type"] == "move_timeout":
                set_status_text(f"Move timed out (not at target)")

    class StopCapturingOperator(base_operator):
        bl_idname = "urmoco.stop_capturing"
        bl_label = "Stop capturing"

        @classmethod
        def poll(cls, context):
            return get_mode() is Mode.SHOOTING

        def on_execute(self, context):
            scheduler.ur_in_q.put({"type": "stop"})
            set_mode(Mode.AWAIT_RESPONSE)

        def on_request(self, context, request):
            if request["type"] == "stop":
                scheduler.ur_in_q.put({"type": "stop_capturing"})
                context.window_manager.urmoco_state.running_in_modal = False
                set_mode(Mode.ON)
                set_status_text("Stopped capturing")
                return {"FINISHED"}

    return [StartCapturingOperator, StopCapturingOperator]
