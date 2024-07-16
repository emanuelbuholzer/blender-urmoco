import logging
import queue

import bpy

from urmoco.scheduler import Scheduler
from urmoco.config import Config
from urmoco.blender.sync import get_urmoco_sync
from urmoco.blender.operators.base_modal_operator import \
    get_synced_modal_operator_class
from urmoco.blender.rig import set_ghost_hidden
from urmoco.blender.state import get_mode, set_mode, set_status_text
from urmoco.mode import Mode

logger = logging.getLogger(__name__)


def get_operators(config: Config, scheduler: Scheduler):
    base_operator = get_synced_modal_operator_class(
        config, scheduler
    )

    class PreferencesConfirmationOperator(base_operator):
        bl_idname = "urmoco.startup"
        bl_label = "Startup"

        @classmethod
        def poll(cls, context):
            return get_mode() is Mode.UNINITIALIZED

        def on_execute(self, context):
            self.report({"INFO"}, "Starting urmoco")

            robot_type = config.config["type"]
            if robot_type == "ur10":
                config.config["robot"][
                    "host"
                ] = context.window_manager.urmoco_preferences.host
                config.config["robot"][
                    "payload"
                ] = context.window_manager.urmoco_preferences.payload
            elif robot_type == "ar4":
                config.config["ar4"]["port"] = context.window_manager.urmoco_preferences.port

            sync = get_urmoco_sync(config, scheduler)
            bpy.app.timers.register(sync)

            scheduler.start_dfmoco_server(config)
            scheduler.start_backend(config)

            scheduler.ur_in_q.put({"type": "hi"})

            set_mode(Mode.AWAIT_RESPONSE)

        def on_request(self, context, request):
            if request["type"] == "startup":
                set_mode(Mode.ON)
                set_status_text("Started urmoco")
                set_ghost_hidden(False)

                def unexpected_load_handler(_a, _b):
                    if get_mode() not in {Mode.OFF, Mode.UNINITIALIZED}:
                        scheduler.ur_in_q.put({"type": "power_off"})
                        logger.warning("urmoco shutdown: new blender file loaded without being powered off")

                bpy.app.handlers.load_pre.append(unexpected_load_handler)

                return {"FINISHED"}

        def invoke(self, context, event):
            return context.window_manager.invoke_props_dialog(self, width=300)

        def draw(self, context):
            robot_type = config.config["type"]
            if robot_type == "ur10":
                self.layout.prop(context.window_manager.urmoco_preferences, "host")
                self.layout.prop(context.window_manager.urmoco_preferences, "payload")
            elif robot_type == "ar4":
                self.layout.prop(context.window_manager.urmoco_preferences, "port")

    return [PreferencesConfirmationOperator]
