import bpy
import logging

from bpy.props import BoolProperty, EnumProperty

from urmoco.blender.state import Mode, set_mode, set_status_text, get_mode
from urmoco.blender.operators.base_modal_operator import get_synced_modal_operator_class


logger = logging.getLogger(__name__)


def get_operators(config, urmoco_in_queue, urmoco_out_queue):
    base_operator = get_synced_modal_operator_class(config, urmoco_in_queue, urmoco_out_queue)

    class StartFreedriveOperator(base_operator):
        bl_idname = 'urmoco.start_freedrive'
        bl_label = 'Start Freedrive'

        frame: EnumProperty(name='Transform',
            items={
                ('LOCAL', 'Local orientation', 'Local'),
                ('GLOBAL', 'Global orientation', 'Global')},
            default='LOCAL')

        free_axis_x: BoolProperty(name='X', default=True)
        free_axis_y: BoolProperty(name='Y', default=True)
        free_axis_z: BoolProperty(name='Z', default=True)
        free_axis_rx: BoolProperty(name='RX', default=True)
        free_axis_ry: BoolProperty(name='RY', default=True)
        free_axis_rz: BoolProperty(name='RZ', default=True)

        @classmethod
        def poll(cls, context):
            return get_mode(context) is Mode.ON

        def on_execute(self, context):
            feature = []
            if self.frame == 'GLOBAL':
                _feature = bpy.data.objects['Armature'].matrix_world
                feature += list(_feature.to_translation())
                feature += list(_feature.to_euler())

            elif self.frame == 'LOCAL':
                _feature = bpy.data.objects['Camera'].matrix_world
                feature += list(_feature.to_translation())
                feature += list(_feature.to_euler())

            urmoco_in_queue.put({
                'type': 'start_freedrive',
                'payload': {
                    'feature': feature,
                    'free_axes': [
                        self.free_axis_x,
                        self.free_axis_y,
                        self.free_axis_z,
                        self.free_axis_rx,
                        self.free_axis_ry,
                        self.free_axis_rz
                    ]
                }
            })
            set_mode(context, Mode.FREEDRIVE)
            set_status_text(context, 'Freedrive started')

        def invoke(self, context, event):
            return context.window_manager.invoke_props_dialog(self, width=300)

        def draw(self, context):
            self.layout.prop(self, 'frame')

            self.layout.label(text='Free axis:')
            row = self.layout.row()

            column_left = row.column()
            column_left.prop(self, 'free_axis_x')
            column_left.prop(self, 'free_axis_y')
            column_left.prop(self, 'free_axis_z')

            column_right = row.column()
            column_right.prop(self, 'free_axis_rx')
            column_right.prop(self, 'free_axis_ry')
            column_right.prop(self, 'free_axis_rz')

    class StopFreedriveOperator(bpy.types.Operator):
        bl_idname = 'urmoco.stop_freedrive'
        bl_label = 'Stop Freedrive'

        @classmethod
        def poll(cls, context):
            return get_mode(context) is Mode.FREEDRIVE

        def execute(self, context):
            urmoco_in_queue.put({'type': 'stop_freedrive'})
            context.window_manager.urmoco_state.running_in_modal = False
            set_mode(context, Mode.ON)
            set_status_text(context, 'Freedrive stopped')
            return {'FINISHED'}

    return [StartFreedriveOperator, StopFreedriveOperator]
