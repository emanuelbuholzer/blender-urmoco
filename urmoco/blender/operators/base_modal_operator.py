import queue
import logging
import bpy
import roboticstoolbox as rtb
import mathutils
import numpy as np
from spatialmath import SE3

from urmoco.blender.sync import handle_reqs

logger = logging.getLogger(__name__)

# Create a robot model that is equal to our rig in Blender
robot_model = rtb.models.DH.UR10()
robot_model.base = robot_model.base @ SE3.Tz(1.2) @ SE3(np.eye(4) * np.array([-1, -1, 1, 1]))


def get_synced_modal_operator_class(config, urmoco_in_queue, urmoco_out_queue):
    class SyncedModalOperator(bpy.types.Operator):
        _timer = None

        def on_request(self, context, request):
            pass

        def modal(self, context, event):
            settings = context.window_manager.urmoco_state
            if not settings.running_in_modal:
                return {'CANCELLED'}

            if event.type.startswith('TIMER'):
                try:
                    request = urmoco_out_queue.get_nowait()

                    if request["type"] == "sync":
                        new_configuration = request["payload"]["joints"]
                        trans_matrix = mathutils.Matrix(np.array(robot_model.fkine(new_configuration)))
                        euler_angles = trans_matrix.to_euler()

                        # First we update the ik control according to the joint angles to go to the new tcp
                        bpy.context.scene.objects["Armature"].pose.bones["IK Control"].rotation_euler[0] = euler_angles[0]
                        bpy.context.scene.objects["Armature"].pose.bones["IK Control"].rotation_euler[1] = euler_angles[1]
                        bpy.context.scene.objects["Armature"].pose.bones["IK Control"].rotation_euler[2] = euler_angles[2]
                        bpy.context.scene.objects["Armature"].pose.bones["IK Control"].location[0] = trans_matrix[0][3]
                        bpy.context.scene.objects["Armature"].pose.bones["IK Control"].location[1] = trans_matrix[1][3]
                        bpy.context.scene.objects["Armature"].pose.bones["IK Control"].location[2] = trans_matrix[2][3]

                        # Once we are at our target, we want to update our joints in order to get the correct IK solution
                        bpy.data.objects["Armature"].pose.bones["Shoulder pan 0"].rotation_euler[1] = new_configuration[
                            0]
                        bpy.data.objects["Armature"].pose.bones["Shoulder lift 0"].rotation_euler[1] = \
                            new_configuration[1]
                        bpy.data.objects["Armature"].pose.bones["Elbow 0"].rotation_euler[1] = new_configuration[2] * -1
                        bpy.data.objects["Armature"].pose.bones["Wrist joint 1 0"].rotation_euler[1] = \
                            new_configuration[3]
                        bpy.data.objects["Armature"].pose.bones["Wrist joint 2 0"].rotation_euler[1] = \
                            new_configuration[4]
                        bpy.data.objects["Armature"].pose.bones["Wrist joint 3 0"].rotation_euler[1] = \
                            new_configuration[5]

                    response = self.on_request(context, request)
                    if response is {'FINISHED'}:
                        return {'CANCELLED'}
                    elif response is not None:
                        return response
                    elif handle_reqs(request, context):
                        return {'CANCELLED'}

                    # While we're in the modal context we want to continue syncing
                    urmoco_in_queue.put({"type": "sync"})

                except queue.Empty:
                    pass

            return {'PASS_THROUGH'}

        def on_execute(self, context):
            pass

        def execute(self, context):
            self.on_execute(context)

            context.window_manager.urmoco_state.running_in_modal = True
            self._timer = context.window_manager.event_timer_add(config.get('robot.sync_interval_seconds'),
                                                                 window=context.window)
            context.window_manager.modal_handler_add(self)

            # Trigger the initial sync
            urmoco_in_queue.put({"type": "sync"})
            return {'RUNNING_MODAL'}

        def cancel(self, context):
            context.window_manager.urmoco_state.running_in_modal = False
            context.window_manager.event_timer_remove(self._timer)

    return SyncedModalOperator
