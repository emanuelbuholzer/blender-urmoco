import bpy

from urmoco.mode import Mode


class URMocoState(bpy.types.PropertyGroup):
    def redraw_panel(self, context):
        if context.area is None:
            return None
        for region in context.area.regions:
            if region.type == "UI":
                region.tag_redraw()
        return None

    status: bpy.props.StringProperty(default="Loaded urmoco addon", update=redraw_panel)
    mode: bpy.props.StringProperty(
        default=Mode.UNINITIALIZED.value, update=redraw_panel
    )
    running_in_modal: bpy.props.BoolProperty(default=False, update=redraw_panel)


def get_mode() -> Mode:
    return Mode(bpy.context.window_manager.urmoco_state.mode)


def set_mode(new_mode: Mode):
    bpy.context.window_manager.urmoco_state.mode = new_mode.value


def get_status_text() -> str:
    return bpy.context.window_manager.urmoco_state.status


def set_status_text(status_text: str):
    bpy.context.window_manager.urmoco_state.status = status_text


def get_running_in_modal() -> bool:
    return bpy.context.window_manager.urmoco_state.running_in_modal
