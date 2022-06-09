import errno

import urx
import urx.ursecmon
import logging
import threading
import socket

logger = logging.getLogger(__name__)


class RobotClient:

    def __init__(self, config):
        self.config = config
        self.robot = None
        threading.excepthook = self.get_threading_excepthook()

    def get_threading_excepthook(self):
        def threading_excepthook(args):
            exc_type = args.exc_type
            exc_value = args.exc_value

            logger.error(f"Threading exception: {exc_type}, {exc_value}")
            logger.error(f"{type(exc_type)}")
            if isinstance(exc_type, socket.timeout):
                self.robot.secmon = urx.ursecmon.SecondaryMonitor(self.config.get('robot.host'))
            else:
                raise Exception(exc_value)

        return threading_excepthook

    def connect(self):
        try:
            self.robot = urx.Robot(self.config.get('robot.host'))
        except (urx.ursecmon.TimeoutException, socket.timeout, OSError) as e:
            if isinstance(e, OSError) and not isinstance(e, socket.timeout):
                if e.errno not in {errno.EHOSTUNREACH, errno.EHOSTDOWN}:
                    raise e
            self.robot = None

    def is_connected(self):
        return self.robot is not None

    def set_tool_center_point(self, tcp):
        self.robot.set_tcp(tcp)

    def set_payload(self, weight_kg):
        self.robot.set_payload(weight_kg)

    def move_to_configuration(self, q):
        self.robot.movej(q, wait=False)

    def get_configuration(self):
        return self.robot.getj()

    def get_joints_distance(self, target_joints):
        return self.robot._get_dist(target_joints, joints=True)

    def start_freedrive(self):
        self.robot.set_freedrive(True, self.config.get('robot.freedrive_timeout_seconds'))

    def stop_freedrive(self):
        self.robot.set_freedrive(False)

    def stop(self):
        self.robot.stop()
