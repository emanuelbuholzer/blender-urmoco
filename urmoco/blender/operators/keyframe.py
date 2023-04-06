import logging

import bpy
import numpy as np

from urmoco.blender.constants import (
    ARMATURE_MODEL,
    BONE_ELBOW,
    BONE_IK_CONTROL,
    BONE_SHOULDER_LIFT,
    BONE_SHOULDER_PAN,
    BONE_WRIST_JOINT_1,
    BONE_WRIST_JOINT_2,
    BONE_WRIST_JOINT_3,
    CONSTRAINT_IK,
    ACTION_ARMATURE_MODEL,
    GROUP_END_EFFECTOR,
)
from urmoco.blender.state import Mode, get_mode, set_mode, set_status_text
from urmoco.config import Config
from urmoco.scheduler import Scheduler

logger = logging.getLogger(__name__)


def get_operators(_config: Config, _scheduler: Scheduler):
    class InsertKeyframeOperator(bpy.types.Operator):
        bl_idname = "urmoco.insert_keyframe"
        bl_label = "Insert keyframe"

        def execute(self, _context):
            shoulder_pan_select_prev = (
                bpy.data.objects[ARMATURE_MODEL]
                .pose.bones[BONE_SHOULDER_PAN]
                .bone.select
            )
            shoulder_lift_select_prev = (
                bpy.data.objects[ARMATURE_MODEL]
                .pose.bones[BONE_SHOULDER_LIFT]
                .bone.select
            )
            elbow_select_prev = (
                bpy.data.objects[ARMATURE_MODEL].pose.bones[BONE_ELBOW].bone.select
            )
            wrist_joint_1_select_prev = (
                bpy.data.objects[ARMATURE_MODEL]
                .pose.bones[BONE_WRIST_JOINT_1]
                .bone.select
            )
            wrist_joint_2_select_prev = (
                bpy.data.objects[ARMATURE_MODEL]
                .pose.bones[BONE_WRIST_JOINT_2]
                .bone.select
            )
            wrist_joint_3_select_prev = (
                bpy.data.objects[ARMATURE_MODEL]
                .pose.bones[BONE_WRIST_JOINT_3]
                .bone.select
            )
            ik_control_select_prev = (
                bpy.data.objects[ARMATURE_MODEL].pose.bones[BONE_IK_CONTROL].bone.select
            )
            bpy.data.objects[ARMATURE_MODEL].pose.bones[
                BONE_SHOULDER_PAN
            ].bone.select = True
            bpy.data.objects[ARMATURE_MODEL].pose.bones[
                BONE_SHOULDER_LIFT
            ].bone.select = True
            bpy.data.objects[ARMATURE_MODEL].pose.bones[BONE_ELBOW].bone.select = True
            bpy.data.objects[ARMATURE_MODEL].pose.bones[
                BONE_WRIST_JOINT_1
            ].bone.select = True
            bpy.data.objects[ARMATURE_MODEL].pose.bones[
                BONE_WRIST_JOINT_2
            ].bone.select = True
            bpy.data.objects[ARMATURE_MODEL].pose.bones[
                BONE_WRIST_JOINT_3
            ].bone.select = True
            bpy.data.objects[ARMATURE_MODEL].pose.bones[
                BONE_IK_CONTROL
            ].bone.select = True

            bpy.ops.anim.keyframe_insert_by_name(type="BUILTIN_KSI_VisualLocRot")
            current_frame = bpy.context.scene.frame_current
            for channel in (
                bpy.data.actions[ACTION_ARMATURE_MODEL]
                .groups[GROUP_END_EFFECTOR]
                .channels
            ):
                if not channel.data_path.endswith("rotation_euler"):
                    continue
                if len(channel.keyframe_points) == 1:
                    continue

                # Find the newly inserted keyframe and the closest to it
                newest_index = 0
                closest_distance = np.abs(
                    channel.keyframe_points[0].co[0] - current_frame
                )
                closest_index = 0
                for i, keyframe in enumerate(channel.keyframe_points[1:]):
                    distance = np.abs(keyframe.co[0] - current_frame)
                    if distance == 0:
                        newest_index = i + 1
                    elif distance < closest_distance:
                        closest_distance = distance
                        closest_index = i + 1
                newest_keyframe = channel.keyframe_points[newest_index]
                print(newest_keyframe.co)
                closest_keyframe = channel.keyframe_points[closest_index]
                print(closest_keyframe.co)

                # Make the keyframes euler compatible
                newest_value = newest_keyframe.co[1]
                closest_value = closest_keyframe.co[1]
                newest_value_inv = newest_value + (2 * np.pi) * (
                    -1 * np.sign(newest_value)
                )
                value_distance = np.abs(newest_value - closest_value)
                value_inv_distance = np.abs(newest_value_inv - closest_value)
                if value_inv_distance < value_distance:
                    newest_keyframe.co[1] = newest_value_inv
                    newest_keyframe.handle_left[1] = newest_keyframe.handle_left[1] + (
                        2 * np.pi
                    ) * (-1 * np.sign(newest_value))
                    newest_keyframe.handle_right[1] = newest_keyframe.handle_right[
                        1
                    ] + (2 * np.pi) * (-1 * np.sign(newest_value))

            set_status_text("Inserted keyframe for robot")

            bpy.data.objects[ARMATURE_MODEL].pose.bones[
                BONE_SHOULDER_PAN
            ].bone.select = shoulder_pan_select_prev
            bpy.data.objects[ARMATURE_MODEL].pose.bones[
                BONE_SHOULDER_LIFT
            ].bone.select = shoulder_lift_select_prev
            bpy.data.objects[ARMATURE_MODEL].pose.bones[
                BONE_ELBOW
            ].bone.select = elbow_select_prev
            bpy.data.objects[ARMATURE_MODEL].pose.bones[
                BONE_WRIST_JOINT_1
            ].bone.select = wrist_joint_1_select_prev
            bpy.data.objects[ARMATURE_MODEL].pose.bones[
                BONE_WRIST_JOINT_2
            ].bone.select = wrist_joint_2_select_prev
            bpy.data.objects[ARMATURE_MODEL].pose.bones[
                BONE_WRIST_JOINT_3
            ].bone.select = wrist_joint_3_select_prev
            bpy.data.objects[ARMATURE_MODEL].pose.bones[
                BONE_IK_CONTROL
            ].bone.select = ik_control_select_prev
            return {"FINISHED"}

    return [InsertKeyframeOperator]
