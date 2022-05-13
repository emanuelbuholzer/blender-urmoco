import logging
import bpy

from urmoco.blender.state import Mode, set_mode, set_status_text, get_mode
from urmoco.blender.operators.base_modal_operator import get_synced_modal_operator_class


logger = logging.getLogger(__name__)


def get_operators(config, urmoco_in_queue, urmoco_out_queue):
    base_operator = get_synced_modal_operator_class(config, urmoco_in_queue, urmoco_out_queue)

    class StartShootingOperator(base_operator):
        bl_idname = "urmoco.start_shooting"
        bl_label = "Start Shooting"

        @classmethod
        def poll(cls, context):
            return get_mode(context) is Mode.ON

        _last_frame = None

        def on_execute(self, context):
            urmoco_in_queue.put({"type": "start_shooting"})
            set_mode(context, Mode.SHOOTING)
            set_status_text(context, "Started shooting")

        def on_request(self, context, request):
            if request["type"] == "stop":
                set_mode(context, Mode.SHOOTING)
                set_status_text(context, "Stopped move before frame")

            if request["type"] == "move_to_frame":
                frame = int(request["payload"]["target_frame"])
                self._last_frame = frame

                bpy.context.scene.frame_set(frame)
                configuration = [
                    bpy.data.objects["Armature"].pose.bones["Shoulder pan 0"].rotation_euler[1],
                    bpy.data.objects["Armature"].pose.bones["Shoulder lift 0"].rotation_euler[1],
                    bpy.data.objects["Armature"].pose.bones["Elbow 0"].rotation_euler[1] * -1,
                    bpy.data.objects["Armature"].pose.bones["Wrist joint 1 0"].rotation_euler[1],
                    bpy.data.objects["Armature"].pose.bones["Wrist joint 2 0"].rotation_euler[1],
                    bpy.data.objects["Armature"].pose.bones["Wrist joint 3 0"].rotation_euler[1]
                ]

                urmoco_in_queue.put({
                    "type": "transfer",
                    "payload": {
                        "target_joints": configuration
                    }
                })
                set_status_text(context, f"Moving to frame {frame}")

            if request["type"] == "move_success":
                set_status_text(context, f"Moved to frame {self._last_frame}")

            if request["type"] == "move_timeout":
                set_status_text(context, f"Move timed out (not at target)")

    class StopShootingOperator(bpy.types.Operator):
        bl_idname = "urmoco.stop_shooting"
        bl_label = "Stop Shooting"

        @classmethod
        def poll(cls, context):
            return get_mode(context) is Mode.SHOOTING

        def execute(self, context):
            urmoco_in_queue.put({"type": "stop_shooting"})
            context.window_manager.urmoco_state.running_in_modal = False
            set_mode(context, Mode.ON)
            set_status_text(context, "Stopped shooting")
            return {'FINISHED'}

    return [StartShootingOperator, StopShootingOperator]
