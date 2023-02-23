import logging
import queue

import bpy

from urmoco.scheduler import Scheduler
from urmoco.config import Config
from urmoco.blender.rig import set_ghost_hidden
from urmoco.blender.state import Mode, set_mode, set_status_text, get_mode

logger = logging.getLogger(__name__)


def handle_reqs(req, scheduler: Scheduler):
    if req["type"] == "error":
        set_mode(bpy.context, Mode.ERROR)
        set_status_text(bpy.context, req["payload"]["message"])
        logger.error(req)
        bpy.ops.urmoco.messagebox("INVOKE_DEFAULT", message=req["payload"]["message"])
        return True

    if req["type"] == "info":
        set_status_text(bpy.context, req["payload"]["status_text"])

    if req["type"] == "locked":
        set_mode(bpy.context, Mode.LOCKED)
        set_status_text(bpy.context, "An emergency stop occured")
        return True

    if req["type"] == "power_off":
        set_mode(bpy.context, Mode.UNINITIALIZED)
        set_status_text(bpy.context, "Robot powered off")
        set_ghost_hidden(True)
        scheduler.terminate_backend()
        scheduler.terminate_dfmoco_server()
        return True

    if req["type"] == "disconnected":
        set_mode(bpy.context, Mode.DISCONNECTED)
        set_status_text(bpy.context, "Robot disconnected or not powered on")
        return True

    if req["type"] == "stop":
        set_mode(bpy.context, Mode.ON)
        set_status_text(bpy.context, "Robot stopped")
        return True


def get_urmoco_sync(config: Config, scheduler: Scheduler):
    def sync():

        if get_mode(bpy.context) is Mode.UNINITIALIZED:
            # Returning none will unregister the timer
            return None
        try:
            if not bpy.context.window_manager.urmoco_state.running_in_modal:
                req = scheduler.ur_out_q.get_nowait()
                handle_reqs(req, scheduler)

        except queue.Empty:
            # No message from the urmoco process, ignoring.
            pass
        return config.get("robot.sync_interval_seconds")

    return sync
