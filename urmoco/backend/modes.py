import logging

logger = logging.getLogger(__name__)


def apply_modes(state, robot_mode, safety_mode, urmoco_out_queue):
    # First we check if the robot is in a locked state, which we can unlock.
    if robot_mode == "IDLE" or safety_mode in {"PROTECTIVE_STOP"}:
        urmoco_out_queue.put({"type": "locked"})

    if robot_mode in {"POWER_ON", "RUNNING"} and safety_mode == "NORMAL":
        urmoco_out_queue.put({"type": "power_on"})

    if robot_mode == "POWER_OFF" and safety_mode == "NORMAL":
        urmoco_out_queue.put({"type": "power_off"})

    # Second we check if the robot is in a locked state, which we cannot unlock.
    if robot_mode not in {"POWER_ON", "POWER_OFF", "RUNNING"} and safety_mode != "NORMAL":
        urmoco_out_queue.put({
            "type": "error",
            "payload": {
                "message": f"Robot cannot be controlled automatically. Robot is in mode {robot_mode} and safety "
                           f"mode {safety_mode}. Please contact support and check the polyscope."
            }
        })

    state["robot_mode"] = robot_mode
    logger.info(f"Robot mode changed to {robot_mode}")

    state["safety_mode"] = safety_mode
    logger.info(f"Safety mode changed to {safety_mode}")
