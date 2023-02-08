import logging
import queue

import bpy

from urmoco.blender.state import Mode, set_mode, set_status_text

logger = logging.getLogger(__name__)


def handle_reqs(req, context):
    if req["type"] == "error":
        set_mode(context, Mode.ERROR)
        set_status_text(context, req["payload"]["message"])
        logger.error(req)
        bpy.ops.urmoco.messagebox("INVOKE_DEFAULT", message=req["payload"]["message"])
        return {"CANCELLED"}

    if req["type"] == "info":
        set_status_text(context, req["payload"]["status_text"])
        return

    elif req["type"] == "locked":
        set_mode(context, Mode.LOCKED)
        set_status_text(context, "An emergency stop occured")
        return {"CANCELLED"}

    elif req["type"] == "power_off":
        set_mode(context, Mode.OFF)
        set_status_text(context, "Robot powered off")
        return {"CANCELLED"}

    elif req["type"] == "disconnected":
        set_mode(context, Mode.DISCONNECTED)
        set_status_text(context, "Robot disconnected or not powered on")
        return {"CANCELLED"}


def get_urmoco_sync(config, urmoco_out_queue):
    def sync():
        try:
            if not bpy.context.window_manager.urmoco_state.running_in_modal:
                req = urmoco_out_queue.get_nowait()
                if req["type"] == "stop":
                    set_mode(bpy.context, Mode.ON)
                    set_status_text(bpy.context, "Movement stopped")

                handle_reqs(req, bpy.context)

        except queue.Empty:
            # No message from the urmoco process, ignoring.
            pass
        return config.get("robot.sync_interval_seconds")

    return sync
