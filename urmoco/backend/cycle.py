import logging
import queue
import time

from urmoco.backend.dfmoco import handle_dfmoco_request
from urmoco.backend.modes import apply_modes
from urmoco.backend.move import handle_move
from urmoco.backend.urmoco import handle_urmoco_request

logger = logging.getLogger(__name__)


def run_cycle(config, state, robot, urmoco_in_queue, dfmoco_in_queue, urmoco_out_queue, dfmoco_out_queue):
    # The duration of each cycle is measured in order to support long
    # operation that could potentially time out.
    cycle_start_time = time.time()

    if state['terminated']:
        time.sleep(1)
        return

    # Handle and apply robot and safety modes
    robot_mode = robot.get_mode()
    safety_mode = robot.get_safety_mode()
    if robot_mode != state["robot_mode"] or safety_mode != state["safety_mode"]:
        apply_modes(state, robot_mode, safety_mode, urmoco_out_queue)

    # Handle incoming urmoco requests from blender
    try:
        urmoco_req = urmoco_in_queue.get_nowait()
        handle_urmoco_request(urmoco_req, state, robot, urmoco_out_queue, dfmoco_out_queue)
    except queue.Empty:
        # There was no urmoco request, we continue
        pass

    # If the robot is moving we need to handle the move, thus check for timeouts and the completion of the move
    if state["move"]["active"]:
        handle_move(config, state, robot, urmoco_out_queue, dfmoco_out_queue)

    # If we're shooting with Dragonframe we need to handle its requests too
    if state["shooting"]:
        try:
            dfmoco_req = dfmoco_in_queue.get_nowait()
            handle_dfmoco_request(dfmoco_req, state, dfmoco_out_queue, urmoco_out_queue)
        except queue.Empty:
            # There was no dfmoco request, we continue
            pass

    # In the end we need to calculate the time we used in the cycle :)
    state["cycle"]["prev_duration_seconds"] = time.time() - cycle_start_time
