import logging

from urmoco.backend.robot import RobotClient

logger = logging.getLogger(__name__)


def handle_move(config, state, robot: RobotClient, ur_out_q, df_out_q):
    target_distance = robot.get_joints_distance(state["move"]["target_joints"])
    robot_type = config.get("type")
    target_distance_threshold = config.get(f"{robot_type}.target_distance_threshold")
    if target_distance < target_distance_threshold:
        logger.debug("Reached target within threshold")
        if not robot.is_steady():
            logger.debug("Robot not steady yet")
        else:
            logger.debug("Steady at target")
            state["move"]["stopping"] = False
            state["move"]["scheduled"] = False
            state["move"]["active"] = False
            state["move"]["target_joints"] = None
            state["move"]["time_elapsed_seconds"] = 0

            response = {
                "type": "move_success",
                "payload": {"frame": state["move"]["target_frame"]},
            }
            ur_out_q.put(response)
            df_out_q.put(response)
    elif target_distance >= target_distance_threshold:
        logger.warning("Robot not within target threshold")
    elif state["move"]["stopping"]:
        if not robot.is_steady():
            logger.debug("Robot not steady yet")
        else:
            state["frame"] = -1
            state["move"]["stopping"] = False
            state["move"]["scheduled"] = False
            state["move"]["active"] = False
            state["move"]["target_joints"] = None
            state["move"]["time_elapsed_seconds"] = 0
            joints = robot.get_configuration()
            ur_out_q.put({"type": "sync", "payload": {"joints": joints}})
            ur_out_q.put({"type": "stop"})
            df_out_q.put({"type": "stop_motor"})
            df_out_q.put({"type": "stop_all"})
            df_out_q.put(
                {"type": "set_frame", "payload": {"current_frame": state["frame"]}}
            )

    state["move"]["time_elapsed_seconds"] += state["cycle"]["prev_duration_seconds"]

    timeout_seconds = config.get(f"{robot_type}.move_timeout_seconds")
    if (
        state["move"]["time_elapsed_seconds"] > timeout_seconds
        and not state["move"]["stopping"]
    ):
        robot.stop()
        state["move"]["stopping"] = True
        state["move"]["active"] = False
        state["move"]["target_joints"] = None
        state["move"]["time_elapsed_seconds"] = 0
        ur_out_q.put({"type": "move_timeout"})
        df_out_q.put({"type": "move_timeout"})
