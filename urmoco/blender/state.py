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
    mode: bpy.props.StringProperty(default=Mode.UNINITIALIZED.value, update=redraw_panel)
    running_in_modal: bpy.props.BoolProperty(default=False, update=redraw_panel)

    baked: bpy.props.BoolProperty(name="Use baked poses", default=False, update=redraw_panel)


def get_mode(context) -> Mode:
    return Mode(context.window_manager.urmoco_state.mode)


def set_mode(context, new_mode: Mode):
    context.window_manager.urmoco_state.mode = new_mode.value


def get_status_text(context) -> str:
    return context.window_manager.urmoco_state.status


def set_status_text(context, status_text: str):
    context.window_manager.urmoco_state.status = status_text
