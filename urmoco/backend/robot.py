import errno
import logging
import rtde_control
import rtde_receive

logger = logging.getLogger(__name__)


class RobotClient:

    def __init__(self, config):
        self.config = config
        self.rtde_r: rtde_receive.RTDEReceiveInterface = None
        self.rtde_c: rtde_control.RTDEControlInterface = None

    def connect(self):
        try:
            self.rtde_r = rtde_receive.RTDEReceiveInterface(self.config.get("robot.host"))
            self.rtde_c = rtde_control.RTDEControlInterface(self.config.get("robot.host"))
        except Exception as e:
            self.rtde_r = None
            self.rtde_c = None
            logger.error(e)
            pass

    def disconnect(self):
        self.rtde_r.disconnect()
        self.rtde_c.disconnect()

    def reconnect(self):
        if not self.rtde_c.isConnected():
            self.rtde_c.reconnect()
        if not self.rtde_r.isConnected():
            self.rtde_r.reconnect()

    def is_connected(self):
        return self.rtde_r is not None and self.rtde_c is not None and self.rtde_r.isConnected() and self.rtde_c.isConnected()

    def set_tool_center_point(self, tcp):
        self.rtde_c.setTcp(list(tcp))

    def set_payload(self, weight_kg):
        self.rtde_c.setPayload(float(weight_kg), [0.0,0.0,0.0])

    def move_to_configuration(self, q):
        self.rtde_c.moveJ(list(q), 0.5, 0.3, True)

    def get_configuration(self):
        return tuple(self.rtde_r.getActualQ())

    def get_joints_distance(self, target):
        q = self.get_configuration()
        dist = 0
        for i in range(6):
            dist += (target[i] - q[i]) ** 2
        return dist ** 0.5

    def start_freedrive(self):
        if not self.rtde_c.teachMode():
            raise Exception("Could not enter freedrive mode")

    def stop_freedrive(self):
        self.rtde_c.endTeachMode()

    def stop(self):
        self.rtde_c.stopJ(2.0)
