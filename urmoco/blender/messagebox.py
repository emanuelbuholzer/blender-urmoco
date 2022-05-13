import bpy


class MessageBox(bpy.types.Operator):
    bl_idname = "urmoco.messagebox"
    bl_label = "URMoco"

    message: bpy.props.StringProperty()

    def execute(self, context):
        self.report({'INFO'}, self.message)
        return {'FINISHED'}

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=400)

    def draw(self, context):
        self.layout.label(text=self.message)
