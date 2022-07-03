import time
import logging
from urmoco.backend.modes import apply_modes
from urmoco.backend.state import get_initial_state

logger = logging.getLogger(__name__)


def handle_urmoco_request(config, urmoco_req, state, robot, dashboard, urmoco_out_queue, dfmoco_out_queue):
    if urmoco_req["type"] == "hi":
        state = get_initial_state()
        robot_mode = dashboard.get_mode()
        safety_mode = dashboard.get_safety_mode()
        apply_modes(state, robot_mode, safety_mode, urmoco_out_queue)

        robot.set_tool_center_point((0, 0, 0.1, 0, 0, 0))
        robot.set_payload(state["payload"])
        time.sleep(1)

    if urmoco_req["type"] == "power_off":
        dashboard.power_off()

    if urmoco_req["type"] == "power_on":
        dashboard.close_popup()
        dashboard.close_safety_popup()
        time.sleep(0.5)
        dashboard.power_on()
        time.sleep(3)
        robot.set_tool_center_point((0, 0, 0.1, 0, 0, 0))
        robot.set_payload(state["payload"])
        time.sleep(1)
        dashboard.release_brakes()
        time.sleep(2)
        joints = robot.get_configuration()
        urmoco_out_queue.put({
            "type": "sync",
            "payload": {
                "joints": joints
            }
        })
        time.sleep(2)
        urmoco_out_queue.put({"type": "power_on"})

    if urmoco_req["type"] == "unlock":
        dashboard.unlock_protective_stop()
        time.sleep(2)
        dashboard.release_brakes()
        time.sleep(3)

    if urmoco_req["type"] == "start_shooting":
        state["shooting"] = True
        state["frame"] = -1

    if urmoco_req["type"] == "stop_shooting":
        state["shooting"] = False
        state["frame"] = -1

    if urmoco_req["type"] == "transfer":
        if not state["move"]["active"]:
            joints = urmoco_req["payload"]["target_joints"]
            robot.move_to_configuration(joints)
            state["move"]["active"] = True
            state["move"]["target_joints"] = joints
            if not state["shooting"]:
                state["frame"] = -1

    if urmoco_req["type"] == "sync":
        joints = robot.get_configuration()
        urmoco_out_queue.put({
            "type": "sync",
            "payload": {
                "joints": joints
            }
        })

    if urmoco_req["type"] == "start_freedrive":
        robot.start_freedrive()

    if urmoco_req["type"] == "stop_freedrive":
        robot.stop_freedrive()

    if urmoco_req["type"] == "stop":
        robot.stop()
        state["frame"] = -1
        state["move"]["active"] = False
        state["move"]["target_joints"] = None
        state["move"]["time_elapsed_seconds"] = 0
        urmoco_out_queue.put({"type": "stop"})
        dfmoco_out_queue.put({
            "type": "set_frame",
            "payload": {
                "current_frame": state["frame"]
            }
        })
