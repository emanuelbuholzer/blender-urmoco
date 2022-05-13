import bpy

from urmoco.blender.state import get_mode, Mode, get_status_text


class URMocoPanel(bpy.types.Panel):
    bl_label = "Universal Robot Motion Control"
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
        if state in {Mode.OFF, Mode.DISCONNECTED}:
            column_left.operator("urmoco.power_on")
            column_left.enabled = state is not Mode.DISCONNECTED
        else:
            column_left.operator("urmoco.power_off")
            column_left.enabled = state == Mode.ON

        column_right = row.column()
        if state is Mode.LOCKED:
            column_right.operator("urmoco.unlock")
        else:
            column_right.enabled = state in {Mode.AWAIT_RESPONSE, Mode.SHOOTING}
            column_right.operator("urmoco.emergency_stop")

        row = self.layout.row()
        if state is Mode.FREEDRIVE:
            row.operator("urmoco.stop_freedrive")
        else:
            row.enabled = state is Mode.ON
            row.operator("urmoco.start_freedrive")

    def draw_pose_section(self, state: Mode):
        self.layout.label(text="Pose:")

        row = self.layout.row()
        row.enabled = state is Mode.ON
        row.operator("urmoco.transfer")
        row.operator("urmoco.sync")

    def draw(self, context):
        self.layout.label(icon='INFO', text=get_status_text(context))

        state = get_mode(context)
        self.draw_shooting_section(state)
        self.draw_robot_section(state)
        self.draw_pose_section(state)
