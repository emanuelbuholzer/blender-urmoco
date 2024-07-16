import bpy
from bpy.props import FloatProperty, StringProperty


def get_preferences_property_group(config):
    class URMocoPreferences(bpy.types.PropertyGroup):
        if config["type"] == "ur10":
            host: StringProperty(name="Robot host", default=config.get("robot.host"))
            payload: FloatProperty(name="Payload (kg)")
        elif config["type"] == "ar4":
            port: StringProperty(name="Robot port", default="/dev/ttyACM0")

    return URMocoPreferences


class Preferences(bpy.types.AddonPreferences):
    bl_idname = "urmoco"

    def draw(self, context):
        layout = self.layout
        layout.prop(context.window_manager.urmoco_preferences, "host")
