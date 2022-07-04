import bpy
from mathutils import Matrix

from urmoco.blender.constants import BONE_SHOULDER_PAN, BONE_SHOULDER_LIFT, BONE_ELBOW, BONE_WRIST_JOINT_1, \
    BONE_WRIST_JOINT_2, BONE_WRIST_JOINT_3, BONE_IK_CONTROL, CONSTRAINT_IK


def apply_q(target_armature, q):

    ik_enabled_prev = bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_3].constraints[CONSTRAINT_IK].enabled
    bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_3].constraints[CONSTRAINT_IK].enabled = False

    bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_PAN].rotation_euler[1] = q[0]
    bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_LIFT].rotation_euler[1] = q[1]
    bpy.data.objects[target_armature].pose.bones[BONE_ELBOW].rotation_euler[1] = q[2] * -1
    bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_1].rotation_euler[1] = q[3]
    bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_2].rotation_euler[1] = q[4]
    bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_3].rotation_euler[1] = q[5]

    shoulder_pan_select_prev = bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_PAN].bone.select
    shoulder_lift_select_prev = bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_LIFT].bone.select
    elbow_select_prev = bpy.data.objects[target_armature].pose.bones[BONE_ELBOW].bone.select
    wrist_joint_1_select_prev = bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_1].bone.select
    wrist_joint_2_select_prev = bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_2].bone.select
    wrist_joint_3_select_prev = bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_3].bone.select

    bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_PAN].bone.select = True
    bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_LIFT].bone.select = True
    bpy.data.objects[target_armature].pose.bones[BONE_ELBOW].bone.select = True
    bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_1].bone.select = True
    bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_2].bone.select = True
    bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_3].bone.select = True

    bpy.ops.pose.visual_transform_apply()

    tail_loc = bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_3].tail
    _head_loc, rot, _scale = bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_3].matrix.decompose()
    bpy.data.objects[target_armature].pose.bones[BONE_IK_CONTROL].matrix = Matrix.LocRotScale(tail_loc, rot, None)

    bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_PAN].bone.select = shoulder_pan_select_prev
    bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_LIFT].bone.select = shoulder_lift_select_prev
    bpy.data.objects[target_armature].pose.bones[BONE_ELBOW].bone.select = elbow_select_prev
    bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_1].bone.select = wrist_joint_1_select_prev
    bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_2].bone.select = wrist_joint_2_select_prev
    bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_3].bone.select = wrist_joint_3_select_prev

    bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_3].constraints[CONSTRAINT_IK].enabled = ik_enabled_prev

def get_q(target_armature):
    shoulder_pan_select_prev = bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_PAN].bone.select
    shoulder_lift_select_prev = bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_LIFT].bone.select
    elbow_select_prev = bpy.data.objects[target_armature].pose.bones[BONE_ELBOW].bone.select
    wrist_joint_1_select_prev = bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_1].bone.select
    wrist_joint_2_select_prev = bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_2].bone.select
    wrist_joint_3_select_prev = bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_3].bone.select

    bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_PAN].bone.select = True
    bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_LIFT].bone.select = True
    bpy.data.objects[target_armature].pose.bones[BONE_ELBOW].bone.select = True
    bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_1].bone.select = True
    bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_2].bone.select = True
    bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_3].bone.select = True

    bpy.ops.pose.visual_transform_apply()

    q = [
        bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_PAN].rotation_euler[1],
        bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_LIFT].rotation_euler[1],
        bpy.data.objects[target_armature].pose.bones[BONE_ELBOW].rotation_euler[1] * -1,
        bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_1].rotation_euler[1],
        bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_2].rotation_euler[1],
        bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_3].rotation_euler[1]
    ]

    bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_PAN].bone.select = shoulder_pan_select_prev
    bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_LIFT].bone.select = shoulder_lift_select_prev
    bpy.data.objects[target_armature].pose.bones[BONE_ELBOW].bone.select = elbow_select_prev
    bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_1].bone.select = wrist_joint_1_select_prev
    bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_2].bone.select = wrist_joint_2_select_prev
    bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_3].bone.select = wrist_joint_3_select_prev

    return q
