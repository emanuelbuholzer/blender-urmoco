import bpy
from bpy.props import StringProperty, FloatProperty


def get_preferences_property_group(config):
    class URMocoPreferences(bpy.types.PropertyGroup):
        host: StringProperty(name="Robot host", default=config.get('robot.host'))
        payload: FloatProperty(name="Payload (kg)")

    return URMocoPreferences


class Preferences(bpy.types.AddonPreferences):
    bl_idname = 'urmoco'

    def draw(self, context):
        layout = self.layout
        layout.prop(context.window_manager.urmoco_preferences, 'host')
