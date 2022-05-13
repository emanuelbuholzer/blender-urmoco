import logging
from .cycle import run_cycle
from .dashboard import DashboardClient
from .robot import RobotClient
import time

logger = logging.getLogger(__name__)


def run(config, urmoco_in_queue, dfmoco_in_queue, urmoco_out_queue, dfmoco_out_queue):

    # Clients to the robots primary tcp interface for sending programs
    # and to the secondary interface for reads and state updates and
    # the dashboard server for polyscope control
    robot = RobotClient(config)
    dashboard = DashboardClient(config)

    # Try connecting to the robot until the robot is reachablee
    robot.connect()
    while not robot.is_connected():
        urmoco_out_queue.put({"type": "disconnected"})
        robot.connect()
        time.sleep(config.get('robot.connect_interval_seconds'))
    logger.info("Connected to the robot")

    # A somewhat educated guess on what the initial state of the robot and the
    # system could be. This initial state gets and needs to be updated during
    # the first cycle.
    state = {
        'shooting': False,
        'frame': -1,
        'robot_mode': None,
        'safety_mode': None,
        'payload': config.get('robot.payload'),
        'move': {
            'active': False,
            'target_joints': None,
            'target_frame': None,
            'time_elapsed_seconds': 0.
        },
        'cycle': {
            'prev_duration_seconds': 0.,
        }
    }

    # The main loop controlling the robot. The loop is designed to be
    # non-blocking and to be run through cycles quickly, as only
    # one request can be processed during one cycle.
    while True:
        try:
            run_cycle(config, state, robot, dashboard, urmoco_in_queue, dfmoco_in_queue, urmoco_out_queue, dfmoco_out_queue)
        except BaseException as exc:
            urmoco_out_queue.put({
                "type": "error",
                "payload": {
                    "message": f"An unexpected error occured: {exc}"
                }
            })
            break

