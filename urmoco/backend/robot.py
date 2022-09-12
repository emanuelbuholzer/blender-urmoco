import logging
import rtde_control
import rtde_receive
import dashboard_client

logger = logging.getLogger(__name__)


class RobotClient:

    def __init__(self, config):
        self.config = config
        self.rtde_r: rtde_receive.RTDEReceiveInterface = None
        self.rtde_c: rtde_control.RTDEControlInterface = None
        self.dashboard_client: dashboard_client.DashboardClient = None

    def connect(self):
        try:
            host = self.config.get("robot.host")
            logger.debug("Trying to connect to dashboard interface")
            self.dashboard_client = dashboard_client.DashboardClient(host)
            self.dashboard_client.connect()
            logger.debug("Trying to connect to rtde receive interface")
            self.rtde_r = rtde_receive.RTDEReceiveInterface(host)
            logger.debug("Trying to connect to rtde control interface")
            self.rtde_c = rtde_control.RTDEControlInterface(host)
            return True

        except Exception as e:
            self.dashboard_client = None
            self.rtde_r = None
            self.rtde_c = None
            logger.debug(f"Connecting to all robot interfaces failed: {e}")
            self.disconnect()
            return False

    def is_connected(self):
        lost_connection = False

        if self.dashboard_client is None:
            lost_connection = True
        elif not self.dashboard_client.isConnected():
            logger.debug("Lost connection to the dashboard interface")
            lost_connection = True

        if self.rtde_r is None:
            lost_connection = True
        elif not self.rtde_r.isConnected():
            logger.debug("Lost connection to the rtde receive interface")
            lost_connection = True

        if self.rtde_c is None:
            lost_connection = True
        elif not self.rtde_c.isConnected():
            logger.debug("Lost connection to the rtde control interface")
            lost_connection = True

        return not lost_connection

    def disconnect(self):
        if self.dashboard_client is not None:
            self.dashboard_client.disconnect()
            logger.debug("Disconnected from the dashboard interface")

        if self.rtde_r is not None:
            self.rtde_r.disconnect()
            logger.debug("Disconnected from the rtde receive interface")

        if self.rtde_c is not None:
            self.rtde_c.disconnect()
            logger.debug("Disconnected from the rtde control interface")

        self.dashboard_client = None
        self.rtde_r = None
        self.rtde_c = None

    def reconnect(self):
        if not self.dashboard_client.isConnected():
            logger.debug("Reconnecting to the dashboard interface")
            self.dashboard_client.connect()
        if not self.rtde_c.isConnected():
            logger.debug("Reconnecting to the rtde receive interface")
            self.rtde_c.reconnect()
        if not self.rtde_r.isConnected():
            logger.debug("Reconnecting to the rtde control interface")
            self.rtde_r.reconnect()

    def set_tool_center_point(self, tcp):
        self.rtde_c.setTcp(list(tcp))

    def set_payload(self, weight_kg):
        self.rtde_c.setPayload(float(weight_kg), [0.0, 0.0, 0.0])

    def move_to_configuration(self, q):
        speed = self.config.get('robot.joint_move_speed')
        acceleration = self.config.get('robot.joint_move_acceleration')
        self.rtde_c.moveJ(list(q), speed, acceleration, True)

    def get_configuration(self):
        return tuple(self.rtde_r.getActualQ())

    def get_joints_distance(self, target):
        q = self.get_configuration()
        dist = 0
        for i in range(6):
            dist += (target[i] - q[i]) ** 2
        return dist ** 0.5

    def get_mode(self):
        return self.rtde_r.getRobotMode()

    def get_safety_mode(self):
        return self.rtde_r.getSafetyMode()

    def start_freedrive(self):
        if not self.rtde_c.teachMode():
            raise Exception("Could not enter freedrive mode")

    def stop_freedrive(self):
        self.rtde_c.endTeachMode()

    def stop(self):
        speed = self.config.get('robot.joint_stop_speeed')
        self.rtde_c.stopJ(speed)

    def unlock_protective_stop(self):
        self.dashboard_client.unlockProtectiveStop()

    def release_brakes(self):
        self.dashboard_client.brakeRelease()

    def power_on(self):
        self.dashboard_client.powerOn()

    def power_off(self):
        self.dashboard_client.powerOff()

    def close_popups(self):
        self.dashboard_client.closePopup()
        self.dashboard_client.closeSafetyPopup()

    def is_steady(self):
        return self.rtde_c.isSteady()
