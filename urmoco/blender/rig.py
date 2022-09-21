import bpy
from mathutils import Matrix

from urmoco.blender.constants import BONE_SHOULDER_PAN, BONE_SHOULDER_LIFT, BONE_ELBOW, BONE_WRIST_JOINT_1, \
    BONE_WRIST_JOINT_2, BONE_WRIST_JOINT_3, BONE_IK_CONTROL, CONSTRAINT_IK, GEO_GHOST_ROOT, GEO_GHOST_SHOULDER_PAN, \
    GEO_GHOST_SHOULDER_LIFT, GEO_GHOST_ELBOW, GEO_GHOST_WRIST_JOINT_1, GEO_GHOST_WRIST_JOINT_2, GEO_GHOST_WRIST_JOINT_3, \
    ARMATURE_GHOST, ARMATURE_MODEL

BONES = [BONE_SHOULDER_PAN, BONE_SHOULDER_LIFT, BONE_ELBOW, BONE_WRIST_JOINT_1, BONE_WRIST_JOINT_2, BONE_WRIST_JOINT_3,
         BONE_IK_CONTROL]


def has_constraints(target_armature, target_bone):
    for constraint in bpy.data.objects[target_armature].pose.bones[target_bone].constraints:
        if constraint.enabled:
            return True
    return False


def apply_q(target_armature, q):
    ik_enabled_prev = bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_3].constraints[
        CONSTRAINT_IK].enabled
    bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_3].constraints[CONSTRAINT_IK].enabled = False

    prev_constraints_enabled = []
    for constraint in bpy.data.objects[target_armature].pose.bones[BONE_IK_CONTROL].constraints:
        prev_constraints_enabled.append(constraint.enabled)
        constraint.enabled = False

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

    model_bones_select_prev = []
    if target_armature == ARMATURE_GHOST:
        for bone in BONES:
            model_bones_select_prev.append(bpy.data.objects[ARMATURE_MODEL].pose.bones[bone].bone.select)
            bpy.data.objects[ARMATURE_MODEL].pose.bones[bone].bone.select = False

    bpy.ops.pose.visual_transform_apply()

    tail_loc = bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_3].tail
    _head_loc, rot, _scale = bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_3].matrix.decompose()
    bpy.data.objects[target_armature].pose.bones[BONE_IK_CONTROL].matrix = Matrix.LocRotScale(tail_loc, rot, None)

    if target_armature == ARMATURE_GHOST:
        for i, bone in enumerate(BONES):
            bpy.data.objects[ARMATURE_MODEL].pose.bones[bone].bone.select = model_bones_select_prev[i]

    bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_PAN].bone.select = shoulder_pan_select_prev
    bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_LIFT].bone.select = shoulder_lift_select_prev
    bpy.data.objects[target_armature].pose.bones[BONE_ELBOW].bone.select = elbow_select_prev
    bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_1].bone.select = wrist_joint_1_select_prev
    bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_2].bone.select = wrist_joint_2_select_prev
    bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_3].bone.select = wrist_joint_3_select_prev

    bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_3].constraints[
        CONSTRAINT_IK].enabled = ik_enabled_prev

    for i, constraint in enumerate(bpy.data.objects[target_armature].pose.bones[BONE_IK_CONTROL].constraints):
        constraint.enabled = prev_constraints_enabled[i]


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

    prev_ik_ctrl_transform = bpy.data.objects[target_armature].pose.bones[BONE_IK_CONTROL].matrix_basis.copy()

    prev_ik_ctrl_constraints_enabled = []
    for constraint in bpy.data.objects[target_armature].pose.bones[BONE_IK_CONTROL].constraints:
        prev_ik_ctrl_constraints_enabled.append(constraint.enabled)

    bpy.ops.pose.visual_transform_apply()

    for constraint in bpy.data.objects[target_armature].pose.bones[BONE_IK_CONTROL].constraints:
        constraint.enabled = False

    q = [
        bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_PAN].rotation_euler[1],
        bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_LIFT].rotation_euler[1],
        bpy.data.objects[target_armature].pose.bones[BONE_ELBOW].rotation_euler[1] * -1,
        bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_1].rotation_euler[1],
        bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_2].rotation_euler[1],
        bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_3].rotation_euler[1]
    ]

    bpy.data.objects[target_armature].pose.bones[BONE_IK_CONTROL].matrix_basis = prev_ik_ctrl_transform

    for i, constraint in enumerate(bpy.data.objects[target_armature].pose.bones[BONE_IK_CONTROL].constraints):
        constraint.enabled = prev_ik_ctrl_constraints_enabled[i]

    bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_PAN].bone.select = shoulder_pan_select_prev
    bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_LIFT].bone.select = shoulder_lift_select_prev
    bpy.data.objects[target_armature].pose.bones[BONE_ELBOW].bone.select = elbow_select_prev
    bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_1].bone.select = wrist_joint_1_select_prev
    bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_2].bone.select = wrist_joint_2_select_prev
    bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_3].bone.select = wrist_joint_3_select_prev

    return q


def set_ghost_hidden(hidden):
    bpy.data.objects[GEO_GHOST_ROOT].hide_viewport = hidden
    bpy.data.objects[GEO_GHOST_SHOULDER_PAN].hide_viewport = hidden
    bpy.data.objects[GEO_GHOST_SHOULDER_LIFT].hide_viewport = hidden
    bpy.data.objects[GEO_GHOST_ELBOW].hide_viewport = hidden
    bpy.data.objects[GEO_GHOST_WRIST_JOINT_1].hide_viewport = hidden
    bpy.data.objects[GEO_GHOST_WRIST_JOINT_2].hide_viewport = hidden
    bpy.data.objects[GEO_GHOST_WRIST_JOINT_3].hide_viewport = hidden
