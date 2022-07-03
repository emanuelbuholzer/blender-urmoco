import logging
from .cycle import run_cycle
import time

from .state import get_initial_state
from .dashboard import DashboardClient
from .robot import RobotClient

logger = logging.getLogger(__name__)


def run(config, urmoco_in_queue, dfmoco_in_queue, urmoco_out_queue, dfmoco_out_queue):
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
    state = get_initial_state()

    # The main loop controlling the robot. The loop is designed to be
    # non-blocking and to be run through cycles quickly, as only
    # one request can be processed during one cycle.
    try:
        while True:
            run_cycle(config, state, robot, dashboard, urmoco_in_queue, dfmoco_in_queue, urmoco_out_queue, dfmoco_out_queue)
    except Exception as exc:
        logger.error(exc)
        urmoco_out_queue.put({
            "type": "error",
            "payload": {
                "message": f"An unexpected error occured: {exc}"
            }
        })
    finally:
        robot.disconnect()
