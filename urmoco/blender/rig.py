import bpy
import logging
from mathutils import Matrix

from urmoco.blender.constants import (ARMATURE_GHOST, ARMATURE_MODEL,
                                      BONE_ELBOW, BONE_IK_CONTROL,
                                      BONE_SHOULDER_LIFT, BONE_SHOULDER_PAN,
                                      BONE_WRIST_JOINT_1, BONE_WRIST_JOINT_2,
                                      BONE_WRIST_JOINT_3, CONSTRAINT_IK,
                                      GEO_GHOST_ELBOW, GEO_GHOST_ROOT,
                                      GEO_GHOST_SHOULDER_LIFT,
                                      GEO_GHOST_SHOULDER_PAN,
                                      GEO_GHOST_WRIST_JOINT_1,
                                      GEO_GHOST_WRIST_JOINT_2,
                                      GEO_GHOST_WRIST_JOINT_3)

logger = logging.getLogger(__name__)

BONES = [
    BONE_SHOULDER_PAN,
    BONE_SHOULDER_LIFT,
    BONE_ELBOW,
    BONE_WRIST_JOINT_1,
    BONE_WRIST_JOINT_2,
    BONE_WRIST_JOINT_3,
    BONE_IK_CONTROL,
]

BONES = [
    "Bone",
    "Bone.001",
    "Bone.002",
    "Bone.003",
    "Bone.004",
    "Bone.005",
    "Bone.006",
    "IK Control"
]

def has_constraints(target_armature, target_bone):
    for constraint in (
            bpy.data.objects[target_armature].pose.bones[target_bone].constraints
    ):
        if constraint.enabled:
            return True
    return False


def apply_q(target_armature, q):
    ik_enabled_prev = (
        bpy.data.objects[target_armature]
        .pose.bones["Bone.005"]
        .constraints[CONSTRAINT_IK]
        .enabled
    )
    bpy.data.objects[target_armature].pose.bones["Bone.005"].constraints[
        CONSTRAINT_IK
    ].enabled = False

    prev_constraints_enabled = []
    for constraint in (
            bpy.data.objects[target_armature].pose.bones["IK Control"].constraints
    ):
        prev_constraints_enabled.append(constraint.enabled)
        constraint.enabled = False

    bpy.data.objects[target_armature].pose.bones["Bone"].rotation_euler[1] = q[0]
    bpy.data.objects[target_armature].pose.bones["Bone.002"].rotation_euler[2] = q[1]
    bpy.data.objects[target_armature].pose.bones["Bone.003"].rotation_euler[2] = q[2]
    bpy.data.objects[target_armature].pose.bones["Bone.006"].rotation_euler[1] = q[3]
    bpy.data.objects[target_armature].pose.bones["Bone.004"].rotation_euler[2] = q[4]
    bpy.data.objects[target_armature].pose.bones["Bone.005"].rotation_euler[1] = q[5]

    shoulder_pan_select_prev = (
        bpy.data.objects[target_armature].pose.bones["Bone"].bone.select
    )
    shoulder_lift_select_prev = (
        bpy.data.objects[target_armature].pose.bones["Bone.002"].bone.select
    )
    elbow_select_prev = (
        bpy.data.objects[target_armature].pose.bones["Bone.003"].bone.select
    )
    wrist_joint_1_select_prev = (
        bpy.data.objects[target_armature].pose.bones["Bone.006"].bone.select
    )
    wrist_joint_2_select_prev = (
        bpy.data.objects[target_armature].pose.bones["Bone.004"].bone.select
    )
    wrist_joint_3_select_prev = (
        bpy.data.objects[target_armature].pose.bones["Bone.005"].bone.select
    )

    bpy.data.objects[target_armature].pose.bones["Bone"].bone.select = True
    bpy.data.objects[target_armature].pose.bones["Bone.002"].bone.select = True
    bpy.data.objects[target_armature].pose.bones["Bone.003"].bone.select = True
    bpy.data.objects[target_armature].pose.bones["Bone.006"].bone.select = True
    bpy.data.objects[target_armature].pose.bones["Bone.004"].bone.select = True
    bpy.data.objects[target_armature].pose.bones["Bone.005"].bone.select = True

    model_bones_select_prev = []
    if target_armature == ARMATURE_GHOST:
        for bone in BONES:
            model_bones_select_prev.append(
                bpy.data.objects[ARMATURE_MODEL].pose.bones[bone].bone.select
            )
            bpy.data.objects[ARMATURE_MODEL].pose.bones[bone].bone.select = False

    bpy.ops.pose.visual_transform_apply()

    tail_loc = bpy.data.objects[target_armature].pose.bones["Bone.005"].tail
    _head_loc, rot, _scale = (
        bpy.data.objects[target_armature]
        .pose.bones["Bone.005"]
        .matrix.decompose()
    )
    bpy.data.objects[target_armature].pose.bones[
        "IK Control"
    ].matrix = Matrix.LocRotScale(tail_loc, rot, None)

    if target_armature == ARMATURE_GHOST:
        for i, bone in enumerate(BONES):
            bpy.data.objects[ARMATURE_MODEL].pose.bones[
                bone
            ].bone.select = model_bones_select_prev[i]

    bpy.data.objects[target_armature].pose.bones["Bone"].bone.select = shoulder_pan_select_prev
    bpy.data.objects[target_armature].pose.bones["Bone.002"].bone.select = shoulder_lift_select_prev
    bpy.data.objects[target_armature].pose.bones["Bone.003"].bone.select = elbow_select_prev
    bpy.data.objects[target_armature].pose.bones["Bone.006"].bone.select = wrist_joint_1_select_prev
    bpy.data.objects[target_armature].pose.bones["Bone.004"].bone.select = wrist_joint_2_select_prev
    bpy.data.objects[target_armature].pose.bones["Bone.005"].bone.select = wrist_joint_3_select_prev

    for i, constraint in enumerate(
            bpy.data.objects[target_armature].pose.bones["IK Control"].constraints
    ):
        constraint.enabled = prev_constraints_enabled[i]

    bpy.data.objects[target_armature].pose.bones["Bone.005"].constraints[
        CONSTRAINT_IK
    ].enabled = ik_enabled_prev


def get_q(target_armature):
    shoulder_pan_select_prev = (
        bpy.data.objects[target_armature].pose.bones["Bone"].bone.select
    )
    shoulder_lift_select_prev = (
        bpy.data.objects[target_armature].pose.bones["Bone.002"].bone.select
    )
    elbow_select_prev = (
        bpy.data.objects[target_armature].pose.bones["Bone.003"].bone.select
    )
    wrist_joint_1_select_prev = (
        bpy.data.objects[target_armature].pose.bones["Bone.006"].bone.select
    )
    wrist_joint_2_select_prev = (
        bpy.data.objects[target_armature].pose.bones["Bone.004"].bone.select
    )
    wrist_joint_3_select_prev = (
        bpy.data.objects[target_armature].pose.bones["Bone.005"].bone.select
    )

    bpy.data.objects[target_armature].pose.bones["Bone"].bone.select = True
    bpy.data.objects[target_armature].pose.bones["Bone.002"].bone.select = True
    bpy.data.objects[target_armature].pose.bones["Bone.003"].bone.select = True
    bpy.data.objects[target_armature].pose.bones["Bone.006"].bone.select = True
    bpy.data.objects[target_armature].pose.bones["Bone.004"].bone.select = True
    bpy.data.objects[target_armature].pose.bones["Bone.005"].bone.select = True

    prev_ik_ctrl_transform = (
        bpy.data.objects[target_armature]
        .pose.bones["IK Control"]
        .matrix_basis.copy()
    )

    prev_ik_ctrl_constraints_enabled = []
    for constraint in (
            bpy.data.objects[target_armature].pose.bones["IK Control"].constraints
    ):
        prev_ik_ctrl_constraints_enabled.append(constraint.enabled)

    bpy.ops.pose.visual_transform_apply()

    for constraint in (
            bpy.data.objects[target_armature].pose.bones["IK Control"].constraints
    ):
        constraint.enabled = False

    q = [
        bpy.data.objects[target_armature].pose.bones["Bone"].rotation_euler[1],
        bpy.data.objects[target_armature].pose.bones["Bone.002"].rotation_euler[2],
        bpy.data.objects[target_armature].pose.bones["Bone.003"].rotation_euler[2],
        bpy.data.objects[target_armature].pose.bones["Bone.006"].rotation_euler[1],
        bpy.data.objects[target_armature].pose.bones["Bone.004"].rotation_euler[2],
        bpy.data.objects[target_armature].pose.bones["Bone.005"].rotation_euler[1]
    ]

    bpy.data.objects[target_armature].pose.bones[
        "IK Control"
    ].matrix_basis = prev_ik_ctrl_transform

    for i, constraint in enumerate(
            bpy.data.objects[target_armature].pose.bones["IK Control"].constraints
    ):
        constraint.enabled = prev_ik_ctrl_constraints_enabled[i]

    bpy.data.objects[target_armature].pose.bones["Bone"].bone.select = shoulder_pan_select_prev
    bpy.data.objects[target_armature].pose.bones["Bone.002"].bone.select = shoulder_lift_select_prev
    bpy.data.objects[target_armature].pose.bones["Bone.003"].bone.select = elbow_select_prev
    bpy.data.objects[target_armature].pose.bones["Bone.006"].bone.select = wrist_joint_1_select_prev
    bpy.data.objects[target_armature].pose.bones["Bone.004"].bone.select = wrist_joint_2_select_prev
    bpy.data.objects[target_armature].pose.bones["Bone.005"].bone.select = wrist_joint_3_select_prev

    return q


def set_ghost_hidden(hidden):
    return
    bpy.data.objects[GEO_GHOST_ROOT].hide_viewport = hidden
    bpy.data.objects[GEO_GHOST_SHOULDER_PAN].hide_viewport = hidden
    bpy.data.objects[GEO_GHOST_SHOULDER_LIFT].hide_viewport = hidden
    bpy.data.objects[GEO_GHOST_ELBOW].hide_viewport = hidden
    bpy.data.objects[GEO_GHOST_WRIST_JOINT_1].hide_viewport = hidden
    bpy.data.objects[GEO_GHOST_WRIST_JOINT_2].hide_viewport = hidden
    bpy.data.objects[GEO_GHOST_WRIST_JOINT_3].hide_viewport = hidden


# def has_constraints(target_armature, target_bone):
#     for constraint in (
#         bpy.data.objects[target_armature].pose.bones[target_bone].constraints
#     ):
#         if constraint.enabled:
#             return True
#     return False
#
#
# def apply_q(target_armature, q):
#     ik_enabled_prev = (
#         bpy.data.objects[target_armature]
#         .pose.bones[BONE_WRIST_JOINT_3]
#         .constraints[CONSTRAINT_IK]
#         .enabled
#     )
#     bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_3].constraints[
#         CONSTRAINT_IK
#     ].enabled = False
#
#     prev_constraints_enabled = []
#     for constraint in (
#         bpy.data.objects[target_armature].pose.bones[BONE_IK_CONTROL].constraints
#     ):
#         prev_constraints_enabled.append(constraint.enabled)
#         constraint.enabled = False
#
#     bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_PAN].rotation_euler[
#         1
#     ] = q[0]
#     bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_LIFT].rotation_euler[
#         1
#     ] = q[1]
#     bpy.data.objects[target_armature].pose.bones[BONE_ELBOW].rotation_euler[1] = (
#         q[2] * -1
#     )
#     bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_1].rotation_euler[
#         1
#     ] = q[3]
#     bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_2].rotation_euler[
#         1
#     ] = q[4]
#     bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_3].rotation_euler[
#         1
#     ] = q[5]
#
#     shoulder_pan_select_prev = (
#         bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_PAN].bone.select
#     )
#     shoulder_lift_select_prev = (
#         bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_LIFT].bone.select
#     )
#     elbow_select_prev = (
#         bpy.data.objects[target_armature].pose.bones[BONE_ELBOW].bone.select
#     )
#     wrist_joint_1_select_prev = (
#         bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_1].bone.select
#     )
#     wrist_joint_2_select_prev = (
#         bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_2].bone.select
#     )
#     wrist_joint_3_select_prev = (
#         bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_3].bone.select
#     )
#
#     bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_PAN].bone.select = True
#     bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_LIFT].bone.select = True
#     bpy.data.objects[target_armature].pose.bones[BONE_ELBOW].bone.select = True
#     bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_1].bone.select = True
#     bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_2].bone.select = True
#     bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_3].bone.select = True
#
#     model_bones_select_prev = []
#     if target_armature == ARMATURE_GHOST:
#         for bone in BONES:
#             model_bones_select_prev.append(
#                 bpy.data.objects[ARMATURE_MODEL].pose.bones[bone].bone.select
#             )
#             bpy.data.objects[ARMATURE_MODEL].pose.bones[bone].bone.select = False
#
#     bpy.ops.pose.visual_transform_apply()
#
#     tail_loc = bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_3].tail
#     _head_loc, rot, _scale = (
#         bpy.data.objects[target_armature]
#         .pose.bones[BONE_WRIST_JOINT_3]
#         .matrix.decompose()
#     )
#     bpy.data.objects[target_armature].pose.bones[
#         BONE_IK_CONTROL
#     ].matrix = Matrix.LocRotScale(tail_loc, rot, None)
#
#     if target_armature == ARMATURE_GHOST:
#         for i, bone in enumerate(BONES):
#             bpy.data.objects[ARMATURE_MODEL].pose.bones[
#                 bone
#             ].bone.select = model_bones_select_prev[i]
#
#     bpy.data.objects[target_armature].pose.bones[
#         BONE_SHOULDER_PAN
#     ].bone.select = shoulder_pan_select_prev
#     bpy.data.objects[target_armature].pose.bones[
#         BONE_SHOULDER_LIFT
#     ].bone.select = shoulder_lift_select_prev
#     bpy.data.objects[target_armature].pose.bones[
#         BONE_ELBOW
#     ].bone.select = elbow_select_prev
#     bpy.data.objects[target_armature].pose.bones[
#         BONE_WRIST_JOINT_1
#     ].bone.select = wrist_joint_1_select_prev
#     bpy.data.objects[target_armature].pose.bones[
#         BONE_WRIST_JOINT_2
#     ].bone.select = wrist_joint_2_select_prev
#     bpy.data.objects[target_armature].pose.bones[
#         BONE_WRIST_JOINT_3
#     ].bone.select = wrist_joint_3_select_prev
#
#     bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_3].constraints[
#         CONSTRAINT_IK
#     ].enabled = ik_enabled_prev
#
#     for i, constraint in enumerate(
#         bpy.data.objects[target_armature].pose.bones[BONE_IK_CONTROL].constraints
#     ):
#         constraint.enabled = prev_constraints_enabled[i]
#
#
# def get_q(target_armature):
#     shoulder_pan_select_prev = (
#         bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_PAN].bone.select
#     )
#     shoulder_lift_select_prev = (
#         bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_LIFT].bone.select
#     )
#     elbow_select_prev = (
#         bpy.data.objects[target_armature].pose.bones[BONE_ELBOW].bone.select
#     )
#     wrist_joint_1_select_prev = (
#         bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_1].bone.select
#     )
#     wrist_joint_2_select_prev = (
#         bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_2].bone.select
#     )
#     wrist_joint_3_select_prev = (
#         bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_3].bone.select
#     )
#
#     bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_PAN].bone.select = True
#     bpy.data.objects[target_armature].pose.bones[BONE_SHOULDER_LIFT].bone.select = True
#     bpy.data.objects[target_armature].pose.bones[BONE_ELBOW].bone.select = True
#     bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_1].bone.select = True
#     bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_2].bone.select = True
#     bpy.data.objects[target_armature].pose.bones[BONE_WRIST_JOINT_3].bone.select = True
#
#     prev_ik_ctrl_transform = (
#         bpy.data.objects[target_armature]
#         .pose.bones[BONE_IK_CONTROL]
#         .matrix_basis.copy()
#     )
#
#     prev_ik_ctrl_constraints_enabled = []
#     for constraint in (
#         bpy.data.objects[target_armature].pose.bones[BONE_IK_CONTROL].constraints
#     ):
#         prev_ik_ctrl_constraints_enabled.append(constraint.enabled)
#
#     bpy.ops.pose.visual_transform_apply()
#
#     for constraint in (
#         bpy.data.objects[target_armature].pose.bones[BONE_IK_CONTROL].constraints
#     ):
#         constraint.enabled = False
#
#     q = [
#         bpy.data.objects[target_armature]
#         .pose.bones[BONE_SHOULDER_PAN]
#         .rotation_euler[1],
#         bpy.data.objects[target_armature]
#         .pose.bones[BONE_SHOULDER_LIFT]
#         .rotation_euler[1],
#         bpy.data.objects[target_armature].pose.bones[BONE_ELBOW].rotation_euler[1] * -1,
#         bpy.data.objects[target_armature]
#         .pose.bones[BONE_WRIST_JOINT_1]
#         .rotation_euler[1],
#         bpy.data.objects[target_armature]
#         .pose.bones[BONE_WRIST_JOINT_2]
#         .rotation_euler[1],
#         bpy.data.objects[target_armature]
#         .pose.bones[BONE_WRIST_JOINT_3]
#         .rotation_euler[1],
#     ]
#
#     bpy.data.objects[target_armature].pose.bones[
#         BONE_IK_CONTROL
#     ].matrix_basis = prev_ik_ctrl_transform
#
#     for i, constraint in enumerate(
#         bpy.data.objects[target_armature].pose.bones[BONE_IK_CONTROL].constraints
#     ):
#         constraint.enabled = prev_ik_ctrl_constraints_enabled[i]
#
#     bpy.data.objects[target_armature].pose.bones[
#         BONE_SHOULDER_PAN
#     ].bone.select = shoulder_pan_select_prev
#     bpy.data.objects[target_armature].pose.bones[
#         BONE_SHOULDER_LIFT
#     ].bone.select = shoulder_lift_select_prev
#     bpy.data.objects[target_armature].pose.bones[
#         BONE_ELBOW
#     ].bone.select = elbow_select_prev
#     bpy.data.objects[target_armature].pose.bones[
#         BONE_WRIST_JOINT_1
#     ].bone.select = wrist_joint_1_select_prev
#     bpy.data.objects[target_armature].pose.bones[
#         BONE_WRIST_JOINT_2
#     ].bone.select = wrist_joint_2_select_prev
#     bpy.data.objects[target_armature].pose.bones[
#         BONE_WRIST_JOINT_3
#     ].bone.select = wrist_joint_3_select_prev
#
#     return q
#
#
# def set_ghost_hidden(hidden):
#     bpy.data.objects[GEO_GHOST_ROOT].hide_viewport = hidden
#     bpy.data.objects[GEO_GHOST_SHOULDER_PAN].hide_viewport = hidden
#     bpy.data.objects[GEO_GHOST_SHOULDER_LIFT].hide_viewport = hidden
#     bpy.data.objects[GEO_GHOST_ELBOW].hide_viewport = hidden
#     bpy.data.objects[GEO_GHOST_WRIST_JOINT_1].hide_viewport = hidden
#     bpy.data.objects[GEO_GHOST_WRIST_JOINT_2].hide_viewport = hidden
#     bpy.data.objects[GEO_GHOST_WRIST_JOINT_3].hide_viewport = hidden
