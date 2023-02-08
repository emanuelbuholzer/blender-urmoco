import bpy

from urmoco import Config
from urmoco.blender.operators.base_modal_operator import \
    get_synced_modal_operator_class
from urmoco.blender.rig import set_ghost_hidden
from urmoco.blender.state import get_mode, set_mode, set_status_text
from urmoco.mode import Mode


def get_operators(config: Config, scheduler, urmoco_in_queue, urmoco_out_queue):
    base_operator = get_synced_modal_operator_class(
        config, urmoco_in_queue, urmoco_out_queue
    )

    class PreferencesConfirmationOperator(base_operator):
        bl_idname = "urmoco.startup"
        bl_label = "Startup"

        @classmethod
        def poll(cls, context):
            return get_mode(context) is Mode.UNINITIALIZED

        def on_execute(self, context):
            self.report({"INFO"}, "Starting urmoco")

            config.config["robot"][
                "host"
            ] = context.window_manager.urmoco_preferences.host
            config.config["robot"][
                "payload"
            ] = context.window_manager.urmoco_preferences.payload

            scheduler.start_dfmoco_server(config)
            scheduler.start_backend(config)

            scheduler.ur_in_q.put({"type": "hi"})

            set_mode(context, Mode.AWAIT_RESPONSE)

        def on_request(self, context, request):
            if request["type"] == "startup":
                set_mode(context, Mode.ON)
                set_status_text(context, "Started urmoco")
                set_ghost_hidden(False)
                return {"FINISHED"}

        def invoke(self, context, event):
            return context.window_manager.invoke_props_dialog(self, width=300)

        def draw(self, context):
            self.layout.prop(context.window_manager.urmoco_preferences, "host")
            self.layout.prop(context.window_manager.urmoco_preferences, "payload")

    return [PreferencesConfirmationOperator]
