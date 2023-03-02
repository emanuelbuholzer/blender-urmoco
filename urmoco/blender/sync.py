import logging
import queue

from urmoco.blender.handlers import handle_reqs
from urmoco.blender.state import Mode, get_mode, get_running_in_modal
from urmoco.config import Config
from urmoco.scheduler import Scheduler

logger = logging.getLogger(__name__)


def get_urmoco_sync(config: Config, scheduler: Scheduler):
    def sync():
        if get_mode() is Mode.UNINITIALIZED:
            # Returning none will unregister the timer
            return None
        try:
            if not get_running_in_modal():
                req = scheduler.ur_out_q.get_nowait()
                handle_reqs(req, scheduler)

        except queue.Empty:
            # No message from the urmoco process, ignoring.
            pass
        return config.get("robot.sync_interval_seconds")

    return sync
