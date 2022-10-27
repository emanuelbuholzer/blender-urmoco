import logging
import numpy as np

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

    if urmoco_req["type"] == "start_capturing":
        state["shooting"] = True
        state["frame"] = -1
        dfmoco_out_queue.put({
            "type": "set_frame",
            "payload": {
                "current_frame": state["frame"]
            }
        })

    if urmoco_req["type"] == "stop_capturing":
        state["shooting"] = False
        state["frame"] = -1
        dfmoco_out_queue.put({
            "type": "set_frame",
            "payload": {
                "current_frame": state["frame"]
            }
        })

    if urmoco_req["type"] == "transfer":
        if not state["move"]["active"]:
            joints = np.rad2deg(list(urmoco_req["payload"]["target_joints"]))
            current_joints = robot.get_configuration()
            target_joints = []
            for i, joint in enumerate(joints):
                upper_mod = joint % 360
                lower_mod = joint % -360
                upper_dist = (current_joints[i] - upper_mod)**2
                lower_dist = (current_joints[i] - lower_mod)**2
                if upper_dist <= lower_dist:
                    target_joints.append(np.deg2rad(upper_mod))
                else:
                    target_joints.append(np.deg2rad(lower_mod))

            robot.move_to_configuration(target_joints)
            state["move"]["active"] = True
            state["move"]["target_joints"] = target_joints
            if 'target_frame' in urmoco_req["payload"].keys():
                state["move"]["target_frame"] = urmoco_req["payload"]["target_frame"]
            if not state["shooting"]:
                state["frame"] = -1
                dfmoco_out_queue.put({
                    "type": "set_frame",
                    "payload": {
                        "current_frame": state["frame"]
                    }
                })
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
        if not state["shooting"]:
            state["frame"] = -1
            dfmoco_out_queue.put({
                "type": "set_frame",
                "payload": {
                    "current_frame": state["frame"]
                }
            })

    if urmoco_req["type"] == "stop_freedrive":
        robot.stop_freedrive()
        if not state["shooting"]:
            state["frame"] = -1
            dfmoco_out_queue.put({
                "type": "set_frame",
                "payload": {
                    "current_frame": state["frame"]
                }
            })

    if urmoco_req["type"] == "stop":
        robot.stop()
        state["move"]["stopping"] = True
        state["frame"] = -1
        dfmoco_out_queue.put({
            "type": "set_frame",
            "payload": {
                "current_frame": state["frame"]
            }
        })
