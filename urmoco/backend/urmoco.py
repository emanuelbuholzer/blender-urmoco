import logging

import time

from urmoco.backend.robot import RobotClient

logger = logging.getLogger(__name__)


def handle_urmoco_request(urmoco_req, state, robot: RobotClient, urmoco_out_queue, dfmoco_out_queue):
    if urmoco_req["type"] == "power_off":
        urmoco_out_queue.put({
            'type': 'info',
            'payload': {
                'status_text': 'Powering off'
            }
        })
        robot.power_off()
        time.sleep(5)

        urmoco_out_queue.put({
            'type': 'info',
            'payload': {
                'status_text': 'Disconnecting'
            }
        })

        robot.disconnect()
        time.sleep(1)
        state['terminated'] = True
        urmoco_out_queue.put({
            "type": "power_off"
        })

    if urmoco_req["type"] == "unlock":
        time.sleep(5)
        robot.unlock_protective_stop()
        time.sleep(3)
        robot.release_brakes()
        urmoco_out_queue.put({"type": "unlock"})

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
        else:
            logger.debug("A move is already active. Ignoring.")

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
        joints = robot.get_configuration()
        urmoco_out_queue.put({
            "type": "sync",
            "payload": {
                "joints": joints
            }
        })
        urmoco_out_queue.put({"type": "stop"})
        dfmoco_out_queue.put({
            "type": "set_frame",
            "payload": {
                "current_frame": state["frame"]
            }
        })
