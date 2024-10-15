import logging

import bpy

from urmoco.blender.rig import set_ghost_hidden
from urmoco.blender.state import Mode, set_mode, set_status_text
from urmoco.scheduler import Scheduler

logger = logging.getLogger(__name__)


def handle_reqs(req, scheduler: Scheduler):
    if req["type"] == "error":
        set_mode(Mode.ERROR)
        set_status_text(req["payload"]["message"])
        logger.error(req)
        bpy.ops.urmoco.messagebox("INVOKE_DEFAULT", message=req["payload"]["message"])
        return True

    if req["type"] == "info":
        set_status_text(req["payload"]["status_text"])

    if req["type"] == "locked":
        set_mode(Mode.LOCKED)
        set_status_text("An emergency stop occured")
        return True

    if req["type"] == "power_off":
        set_mode(Mode.UNINITIALIZED)
        set_status_text("Robot powered off")
        set_ghost_hidden(True)
        scheduler.terminate_backend()
        scheduler.terminate_dfmoco_server()
        return True

    if req["type"] == "disconnected":
        set_mode(Mode.DISCONNECTED)
        set_status_text("Robot disconnected or not powered on")
        return True

    if req["type"] == "stop":
        set_mode(Mode.ON)
        set_status_text("Robot stopped")
        return True
