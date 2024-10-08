import logging
import time

import numpy as np

from urmoco.backend.robot import RobotClient

logger = logging.getLogger(__name__)


def handle_urmoco_request(
    urmoco_req, state, robot: RobotClient, urmoco_out_queue, dfmoco_out_queue
):
    if urmoco_req["type"] == "calibration":
        robot.calibrate()

    if urmoco_req["type"] == "power_off":
        urmoco_out_queue.put(
            {"type": "info", "payload": {"status_text": "Powering off"}}
        )
        robot.power_off()
        time.sleep(5)

        urmoco_out_queue.put(
            {"type": "info", "payload": {"status_text": "Disconnecting"}}
        )

        robot.disconnect()
        time.sleep(1)
        state["terminated"] = True
        urmoco_out_queue.put({"type": "power_off"})

    if urmoco_req["type"] == "unlock":
        time.sleep(5)
        robot.unlock_protective_stop()
        time.sleep(3)
        # TODO: HUH?
        # robot.release_brakes()
        urmoco_out_queue.put({"type": "unlock"})

    if urmoco_req["type"] == "start_capturing":
        state["shooting"] = True
        state["frame"] = -1
        dfmoco_out_queue.put(
            {"type": "set_frame", "payload": {"current_frame": state["frame"]}}
        )

    if urmoco_req["type"] == "stop_capturing":
        state["shooting"] = False
        state["frame"] = -1
        dfmoco_out_queue.put(
            {"type": "set_frame", "payload": {"current_frame": state["frame"]}}
        )

    if urmoco_req["type"] == "transfer":
        target_joints = np.rad2deg(list(urmoco_req["payload"]["target_joints"]))

        if "target_frame" in urmoco_req["payload"].keys():
            state["move"]["target_frame"] = urmoco_req["payload"]["target_frame"]
            dfmoco_out_queue.put(
                {
                    "type": "move_to_frame",
                    "payload": {"target_frame": state["move"]["target_frame"]},
                }
            )
        if not state["shooting"]:
            state["frame"] = -1
            dfmoco_out_queue.put(
                {"type": "set_frame", "payload": {"current_frame": state["frame"]}}
            )

        robot.move_to_configuration(np.deg2rad(target_joints))

    if urmoco_req["type"] == "sync":
        joints = robot.get_configuration()
        urmoco_out_queue.put({"type": "sync", "payload": {"joints": joints}})

    if urmoco_req["type"] == "start_freedrive":
        robot.start_freedrive()
        if not state["shooting"]:
            state["frame"] = -1
            dfmoco_out_queue.put(
                {"type": "set_frame", "payload": {"current_frame": state["frame"]}}
            )

    if urmoco_req["type"] == "stop_freedrive":
        robot.stop_freedrive()
        if not state["shooting"]:
            state["frame"] = -1
            dfmoco_out_queue.put(
                {"type": "set_frame", "payload": {"current_frame": state["frame"]}}
            )

    if urmoco_req["type"] == "stop":
        if state["move"]["active"]:
            robot.stop()
            state["move"]["stopping"] = True
            state["frame"] = -1
            dfmoco_out_queue.put(
                {"type": "set_frame", "payload": {"current_frame": state["frame"]}}
            )
        else:
            urmoco_out_queue.put({"type": "stop"})
