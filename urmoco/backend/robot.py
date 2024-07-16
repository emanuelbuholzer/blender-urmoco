import logging
from urmoco.backend.ar4 import RobotClientAR4
from urmoco.backend.ur10 import RobotClientUR10
from urmoco.capabilities import CAP_FREEDRIVE, CAP_BRAKE, CAP_POWER

logger = logging.getLogger(__name__)


class RobotClient:
    def __init__(self, config, state, urmoco_out_queue):
        self.config = config
        if config["type"] == "ar4":
            self.comm = RobotClientAR4(config, state, urmoco_out_queue)
        elif config["type"] == "ur10":
            self.comm = RobotClientUR10(config, state, urmoco_out_queue)

    def connect(self):
        return self.comm.connect()

    def is_connected(self):
        return self.comm.is_connected()

    def disconnect(self):
        self.comm.disconnect()

    def reconnect(self):
        self.comm.reconnect()

    def move_to_configuration(self, q):
        self.comm.move_to_configuration(q)

    def get_configuration(self):
        return self.comm.get_configuration()

    def get_joints_distance(self, target):
        q = self.get_configuration()
        dist = 0
        for i in range(6):
            dist += (target[i] - q[i]) ** 2
        return dist ** 0.5

    def get_mode(self):
        return self.comm.get_mode()

    def get_safety_mode(self):
        return self.comm.get_safety_mode()

    def start_freedrive(self):
        robot_type = self.config["type"]
        if CAP_FREEDRIVE not in self.config[robot_type]["capabilities"]:
            logger.error("NO CAP FREEDRIVE")
            return
        self.comm.start_freedrive()

    def stop_freedrive(self):
        robot_type = self.config["type"]
        if CAP_FREEDRIVE not in self.config[robot_type]["capabilities"]:
            logger.error("NO CAP FREEDRIVE")
            return
        self.comm.stop_freedrive()

    def stop(self):
        self.comm.stop()

    def unlock_protective_stop(self):
        robot_type = self.config["type"]
        if CAP_BRAKE not in self.config[robot_type]["capabilities"]:
            logger.error("NO CAP BRAKE")
            return
        self.comm.unlock_protective_stop()

    def power_off(self):
        robot_type = self.config["type"]
        if CAP_POWER not in self.config[robot_type]["capabilities"]:
            logger.error("NO CAP POWER")
            return
        self.comm.power_off()

    def is_steady(self):
        return self.comm.is_steady()
