import bpy
from mathutils import Matrix

from urmoco.blender.constants import (
    ARMATURE_GHOST,
    ARMATURE_MODEL,
    BONE_ELBOW,
    BONE_IK_CONTROL,
    BONE_SHOULDER_LIFT,
    BONE_SHOULDER_PAN,
    BONE_WRIST_JOINT_1,
    BONE_WRIST_JOINT_2,
    BONE_WRIST_JOINT_3,
    CONSTRAINT_IK,
    GEO_GHOST_ELBOW,
    GEO_GHOST_ROOT,
    GEO_GHOST_SHOULDER_LIFT,
    GEO_GHOST_SHOULDER_PAN,
    GEO_GHOST_WRIST_JOINT_1,
    GEO_GHOST_WRIST_JOINT_2,
    GEO_GHOST_WRIST_JOINT_3,
    CONSTRAINT_COPY_LOCATION,
    CONSTRAINT_COPY_ROTATION,
)

BONES = [
    BONE_SHOULDER_PAN,
    BONE_SHOULDER_LIFT,
    BONE_ELBOW,
    BONE_WRIST_JOINT_1,
    BONE_WRIST_JOINT_2,
    BONE_WRIST_JOINT_3,
    BONE_IK_CONTROL,
]


def pose_bone(target_armature, target_bone):
    return bpy.data.objects[target_armature].pose.bones[target_bone]


def pose_bone_constraint(target_armature, target_bone, target_constraint):
    return pose_bone(target_armature, target_bone).constraints[target_constraint]


def has_constraints(target_armature, target_bone):
    for constraint in pose_bone(target_armature, target_bone).constraints:
        if constraint.enabled:
            return True
    return False


def select_bones(target_armature, target_bones):
    for target_bone in target_bones:
        pose_bone(target_armature, target_bone).bone.select = True


def apply_q(target_armature, q):
    ik_constraint = pose_bone_constraint(target_armature, BONE_WRIST_JOINT_3, CONSTRAINT_IK)
    ik_enabled_prev = ik_constraint.enabled
    ik_constraint.enabled = False

    prev_constraints_enabled = []
    for constraint in pose_bone(target_armature, BONE_IK_CONTROL).constraints:
        prev_constraints_enabled.append(constraint.enabled)
        constraint.enabled = False

    if target_armature == ARMATURE_MODEL:
        pose_bone_constraint(ARMATURE_MODEL, BONE_IK_CONTROL, CONSTRAINT_COPY_LOCATION).enabled = True
        pose_bone_constraint(ARMATURE_MODEL, BONE_IK_CONTROL, CONSTRAINT_COPY_ROTATION).enabled = True

    pose_bone(target_armature, BONE_SHOULDER_PAN).rotation_euler[1] = q[0]
    pose_bone(target_armature, BONE_SHOULDER_LIFT).rotation_euler[1] = q[1]
    pose_bone(target_armature, BONE_ELBOW).rotation_euler[1] = q[2] * -1
    pose_bone(target_armature, BONE_WRIST_JOINT_1).rotation_euler[1] = q[3]
    pose_bone(target_armature, BONE_WRIST_JOINT_2).rotation_euler[1] = q[4]
    pose_bone(target_armature, BONE_WRIST_JOINT_3).rotation_euler[1] = q[5]

    shoulder_pan_select_prev = pose_bone(target_armature, BONE_SHOULDER_PAN).bone.select
    shoulder_lift_select_prev = pose_bone(target_armature, BONE_SHOULDER_LIFT).bone.select
    elbow_select_prev = pose_bone(target_armature, BONE_ELBOW).bone.select
    wrist_joint_1_select_prev = pose_bone(target_armature, BONE_WRIST_JOINT_1).bone.select
    wrist_joint_2_select_prev = pose_bone(target_armature, BONE_WRIST_JOINT_2).bone.select
    wrist_joint_3_select_prev = pose_bone(target_armature, BONE_WRIST_JOINT_3).bone.select
    ik_control_select_prev = pose_bone(target_armature, BONE_IK_CONTROL).bone.select

    select_bones(target_armature, BONES)

    model_bones_select_prev = []
    if target_armature == ARMATURE_GHOST:
        for bone_id in BONES:
            p_bone = pose_bone(ARMATURE_MODEL, bone_id)
            model_bones_select_prev.append(p_bone.bone.select)
            p_bone.bone.select = False

    bpy.ops.pose.visual_transform_apply()

    if target_armature == ARMATURE_MODEL:
        pose_bone_constraint(ARMATURE_MODEL, BONE_IK_CONTROL, CONSTRAINT_COPY_LOCATION).enabled = False
        pose_bone_constraint(ARMATURE_MODEL, BONE_IK_CONTROL, CONSTRAINT_COPY_ROTATION).enabled = False
    else:
        wrist_joint_3 = pose_bone(ARMATURE_GHOST, BONE_WRIST_JOINT_3)
        tail_loc = wrist_joint_3.tail
        rot = wrist_joint_3.matrix.to_euler()

        pose_bone(ARMATURE_GHOST, BONE_IK_CONTROL).matrix = Matrix.LocRotScale(tail_loc, rot, None)

    if target_armature == ARMATURE_GHOST:
        for i, bone_id in enumerate(BONES):
            pose_bone(ARMATURE_MODEL, bone_id).bone.select = model_bones_select_prev[i]

    pose_bone(target_armature, BONE_SHOULDER_PAN).bone.select = shoulder_pan_select_prev
    pose_bone(target_armature, BONE_SHOULDER_LIFT).bone.select = shoulder_lift_select_prev
    pose_bone(target_armature, BONE_ELBOW).bone.select = elbow_select_prev
    pose_bone(target_armature, BONE_WRIST_JOINT_1).bone.select = wrist_joint_1_select_prev
    pose_bone(target_armature, BONE_WRIST_JOINT_2).bone.select = wrist_joint_2_select_prev
    pose_bone(target_armature, BONE_WRIST_JOINT_3).bone.select = wrist_joint_3_select_prev
    pose_bone(target_armature, BONE_IK_CONTROL).bone.select = ik_control_select_prev

    pose_bone_constraint(target_armature, BONE_WRIST_JOINT_3, CONSTRAINT_IK).enabled = ik_enabled_prev

    for i, constraint in enumerate(pose_bone(target_armature, BONE_IK_CONTROL).constraints):
        constraint.enabled = prev_constraints_enabled[i]


def get_q(target_armature):
    shoulder_pan_select_prev = pose_bone(target_armature, BONE_SHOULDER_PAN).bone.select
    shoulder_lift_select_prev = pose_bone(target_armature, BONE_SHOULDER_LIFT).bone.select
    elbow_select_prev = pose_bone(target_armature, BONE_ELBOW).bone.select
    wrist_joint_1_select_prev = pose_bone(target_armature, BONE_WRIST_JOINT_1).bone.select
    wrist_joint_2_select_prev = pose_bone(target_armature, BONE_WRIST_JOINT_2).bone.select
    wrist_joint_3_select_prev = pose_bone(target_armature, BONE_WRIST_JOINT_3).bone.select

    bones_without_ik_control = list(filter(lambda bone: bone != BONE_IK_CONTROL, BONES))
    select_bones(target_armature, bones_without_ik_control)

    prev_ik_ctrl_transform = (
        bpy.data.objects[target_armature]
        .pose.bones[BONE_IK_CONTROL]
        .matrix_basis.copy()
    )

    prev_ik_ctrl_constraints_enabled = []
    for constraint in (
        bpy.data.objects[target_armature].pose.bones[BONE_IK_CONTROL].constraints
    ):
        prev_ik_ctrl_constraints_enabled.append(constraint.enabled)

    bpy.ops.pose.visual_transform_apply()

    for constraint in (
        bpy.data.objects[target_armature].pose.bones[BONE_IK_CONTROL].constraints
    ):
        constraint.enabled = False

    q = [
        bpy.data.objects[target_armature]
        .pose.bones[BONE_SHOULDER_PAN]
        .rotation_euler[1],
        bpy.data.objects[target_armature]
        .pose.bones[BONE_SHOULDER_LIFT]
        .rotation_euler[1],
        bpy.data.objects[target_armature].pose.bones[BONE_ELBOW].rotation_euler[1] * -1,
        bpy.data.objects[target_armature]
        .pose.bones[BONE_WRIST_JOINT_1]
        .rotation_euler[1],
        bpy.data.objects[target_armature]
        .pose.bones[BONE_WRIST_JOINT_2]
        .rotation_euler[1],
        bpy.data.objects[target_armature]
        .pose.bones[BONE_WRIST_JOINT_3]
        .rotation_euler[1],
    ]

    bpy.data.objects[target_armature].pose.bones[
        BONE_IK_CONTROL
    ].matrix_basis = prev_ik_ctrl_transform

    for i, constraint in enumerate(
        bpy.data.objects[target_armature].pose.bones[BONE_IK_CONTROL].constraints
    ):
        constraint.enabled = prev_ik_ctrl_constraints_enabled[i]

    bpy.data.objects[target_armature].pose.bones[
        BONE_SHOULDER_PAN
    ].bone.select = shoulder_pan_select_prev
    bpy.data.objects[target_armature].pose.bones[
        BONE_SHOULDER_LIFT
    ].bone.select = shoulder_lift_select_prev
    bpy.data.objects[target_armature].pose.bones[
        BONE_ELBOW
    ].bone.select = elbow_select_prev
    bpy.data.objects[target_armature].pose.bones[
        BONE_WRIST_JOINT_1
    ].bone.select = wrist_joint_1_select_prev
    bpy.data.objects[target_armature].pose.bones[
        BONE_WRIST_JOINT_2
    ].bone.select = wrist_joint_2_select_prev
    bpy.data.objects[target_armature].pose.bones[
        BONE_WRIST_JOINT_3
    ].bone.select = wrist_joint_3_select_prev

    return q


def set_ghost_hidden(hidden):
    bpy.data.objects[GEO_GHOST_ROOT].hide_viewport = hidden
    bpy.data.objects[GEO_GHOST_SHOULDER_PAN].hide_viewport = hidden
    bpy.data.objects[GEO_GHOST_SHOULDER_LIFT].hide_viewport = hidden
    bpy.data.objects[GEO_GHOST_ELBOW].hide_viewport = hidden
    bpy.data.objects[GEO_GHOST_WRIST_JOINT_1].hide_viewport = hidden
    bpy.data.objects[GEO_GHOST_WRIST_JOINT_2].hide_viewport = hidden
    bpy.data.objects[GEO_GHOST_WRIST_JOINT_3].hide_viewport = hidden
