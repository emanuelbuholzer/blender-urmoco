import bpy

from urmoco.blender.state import get_mode, Mode, get_status_text


class URMocoPanel(bpy.types.Panel):
    bl_label = "urmoco"
    bl_idname = "PANEL_PT_URMOCO"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Motion Control"

    def draw_initialised(self, state: Mode):
        self.layout.label(text="Capturing:")
        row = self.layout.row()
        if state is Mode.SHOOTING:
            row.operator("urmoco.stop_capturing")
        else:
            row.operator("urmoco.start_capturing")
            row.enabled = state is Mode.ON

        self.layout.label(text="Movement:")
        row = self.layout.row()
        row.scale_y = 2
        row.enabled = state in {Mode.MOVING, Mode.SHOOTING}
        row.operator("urmoco.emergency_stop")

        self.layout.label(text="Pose:")
        row = self.layout.row()
        if state is Mode.FREEDRIVE:
            row.operator("urmoco.stop_freedrive")
        else:
            row.enabled = state is Mode.ON
            row.operator("urmoco.start_freedrive")

        row = self.layout.row()
        row.enabled = state is Mode.ON
        row.operator("urmoco.transfer")
        row.operator("urmoco.sync")

        self.layout.label(text="Robot:")
        row = self.layout.row()
        row.operator("urmoco.power_off")

    def draw_uninitialised(self, context):
        self.layout.operator('urmoco.startup')

    def draw(self, context):
        state = get_mode(context)

        if context.mode != 'POSE':
            self.layout.label(icon='INFO', text="Please use urmoco in pose mode")
            return

        self.layout.label(icon='INFO', text=get_status_text(context))

        if state is Mode.UNINITIALIZED:
            self.draw_uninitialised(context)
            pass
        else:
            self.draw_initialised(state)
