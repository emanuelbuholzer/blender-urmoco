import logging

logger = logging.getLogger(__name__)

ROBOT_MODE_NO_CONTROLLER = -1
ROBOT_MODE_DISCONNECTED = 0
ROBOT_MODE_CONFIRM_SAFETY = 1
ROBOT_MODE_BOOTING = 2
ROBOT_MODE_POWER_OFF = 3
ROBOT_MODE_POWER_ON = 4
ROBOT_MODE_IDLE = 5
ROBOT_MODE_BACKDRIVE = 6
ROBOT_MODE_RUNNING = 7
ROBOT_MODE_UPDATING_FIRMWARE = 8

SAFETY_MODE_NORMAL_MODE = 1
SAFETY_MODE_REDUCED_MODE = 2
SAFETY_MODE_PROTECTIVE_STOPPED = 3
SAFETY_MODE_RECOVERY_MODE = 4
SAFETY_MODE_SAFEGUARD_STOPPED = 5
SAFETY_MODE_SYSTEM_EMERGENCY_STOPPED = 6
SAFETY_MODE_ROBOT_EMERGENCY_STOPPED = 7
SAFETY_MODE_EMERGENCY_STOPPED = 8
SAFETY_MODE_VIOLATION = 9
SAFETY_MODE_FAULT = 10
SAFETY_MODE_STOPPED_DUE_TO_SAFETY = 11


def apply_modes(state, robot_mode, safety_mode, urmoco_out_queue):
    # First we check if the robot is in a locked state, which we can unlock.
    if robot_mode == ROBOT_MODE_IDLE or safety_mode == SAFETY_MODE_PROTECTIVE_STOPPED:
        urmoco_out_queue.put({"type": "locked"})

    if (
        robot_mode in {ROBOT_MODE_POWER_ON, ROBOT_MODE_RUNNING}
        and safety_mode == SAFETY_MODE_NORMAL_MODE
    ):
        urmoco_out_queue.put({"type": "power_on"})

    if robot_mode == ROBOT_MODE_POWER_OFF and safety_mode == SAFETY_MODE_NORMAL_MODE:
        urmoco_out_queue.put({"type": "power_off"})

    # Second we check if the robot is in a locked state, which we cannot unlock.
    if (
        robot_mode
        not in {ROBOT_MODE_POWER_ON, ROBOT_MODE_POWER_OFF, ROBOT_MODE_RUNNING}
        and safety_mode != SAFETY_MODE_NORMAL_MODE
    ):
        urmoco_out_queue.put(
            {
                "type": "error",
                "payload": {
                    "message": f"Robot cannot be controlled automatically. Robot is in mode {robot_mode} and safety "
                    f"mode {safety_mode}. Please contact support and check the polyscope."
                },
            }
        )

    state["robot_mode"] = robot_mode
    logger.info(f"Robot mode changed to {robot_mode}")

    state["safety_mode"] = safety_mode
    logger.info(f"Safety mode changed to {safety_mode}")
