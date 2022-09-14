import logging
import queue

import bpy

from urmoco.blender.constants import ARMATURE_MODEL, ARMATURE_GHOST
from urmoco.blender.rig import apply_q
from urmoco.blender.sync import handle_reqs

logger = logging.getLogger(__name__)


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
                        apply_q(ARMATURE_GHOST, new_configuration)

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
