import bpy

from urmoco.blender.state import get_mode, Mode, get_status_text


class URMocoPanel(bpy.types.Panel):
    bl_label = "urmoco"
    bl_idname = "PANEL_PT_URMOCO"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Motion Control"

    def draw_shooting_section(self, state: Mode):
        self.layout.label(text="Shooting:")

        row = self.layout.row()
        if state is Mode.SHOOTING:
            row.operator("urmoco.stop_shooting")
        else:
            row.operator("urmoco.start_shooting")
            row.enabled = state is Mode.ON

    def draw_robot_section(self, state: Mode):
        self.layout.label(text="Robot:")

        row = self.layout.row()

        column_left = row.column()
        column_left.operator("urmoco.power_off")

        column_right = row.column()
        if state is Mode.LOCKED:
            column_right.operator("urmoco.unlock")
        else:
            column_right.enabled = state in {Mode.MOVING, Mode.SHOOTING}
            column_right.operator("urmoco.emergency_stop")

        row = self.layout.row()
        if state is Mode.FREEDRIVE:
            row.operator("urmoco.stop_freedrive")
        else:
            row.enabled = state is Mode.ON
            row.operator("urmoco.start_freedrive")

    def draw_pose_section(self, context, state: Mode):
        self.layout.label(text="Pose:")

        row = self.layout.row()
        row.enabled = state is Mode.ON
        row.operator("urmoco.transfer")
        row.operator("urmoco.sync")

        row = self.layout.row()
        row.prop(context.window_manager.urmoco_state, "baked")

    def draw_preferences_confirmation(self, context):
        self.layout.operator('urmoco.startup')

    def draw(self, context):
        state = get_mode(context)
        self.layout.label(icon='INFO', text=get_status_text(context))
        if state is Mode.UNINITIALIZED:
            self.draw_preferences_confirmation(context)
            pass
        else:
            self.draw_shooting_section(state)
            self.draw_robot_section(state)
            self.draw_pose_section(context, state)
