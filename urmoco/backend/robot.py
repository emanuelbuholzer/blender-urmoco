import logging
import time

import dashboard_client
import rtde_control
import rtde_receive

logger = logging.getLogger(__name__)


class RobotClient:
    def __init__(self, config, state, urmoco_out_queue):
        self.config = config
        self.urmoco_out_queue = urmoco_out_queue
        self.state = state
        self.rtde_r: rtde_receive.RTDEReceiveInterface = None
        self.rtde_c: rtde_control.RTDEControlInterface = None
        self.dashboard_client: dashboard_client.DashboardClient = None

    def connect(self):
        try:
            host = self.config.get("robot.host")

            logger.debug("Trying to connect to dashboard interface")
            self.urmoco_out_queue.put(
                {"type": "info", "payload": {"status_text": "Connecting to the robot"}}
            )

            self.dashboard_client = dashboard_client.DashboardClient(host)
            self.dashboard_client.connect()
            self.urmoco_out_queue.put(
                {"type": "info", "payload": {"status_text": "Connected"}}
            )

            time.sleep(1)

            # If the robot is not off, there's a likely chance of the rtde_control script
            # being installed on the robot. If the script is running or paused, we have no
            # way to reconnect. Sadly there's no way with ur_rtde to determine if there's
            # a script uploaded in advance, so this is our best guess for not running into
            # any issues. If the rtde_control script was paused and we were to connect, even
            # though the arm is powered off, we'll likely crash the backend process...
            #
            # ... so let's hope for the best :)
            robot_mode = str(self.dashboard_client.robotmode()).strip()
            logger.debug(f"Robot in {robot_mode}")
            if robot_mode != "Robotmode: POWER_OFF":
                raise Exception(f"Robot in unknown mode. Cannot start urmoco.")

            self.dashboard_client.closePopup()
            self.dashboard_client.closeSafetyPopup()
            time.sleep(0.5)

            self.urmoco_out_queue.put(
                {"type": "info", "payload": {"status_text": "Powering on"}}
            )
            self.dashboard_client.powerOn()
            time.sleep(5)

            # Sadly we can't release the brakes after we've set the payload.
            # We have to release the brakes before that, otherwise our rtde_control
            # script will be paused and then everything is over.
            self.urmoco_out_queue.put(
                {"type": "info", "payload": {"status_text": "Releasing brakes"}}
            )
            self.dashboard_client.brakeRelease()
            time.sleep(2)

            # Now let's setup the rtde interfaces. Let's hope our backend won't crash.
            # If it did: read all the comments above.
            self.urmoco_out_queue.put(
                {"type": "info", "payload": {"status_text": "Setup robot control"}}
            )
            logger.debug("Trying to connect to rtde receive interface")
            self.rtde_r = rtde_receive.RTDEReceiveInterface(host)
            logger.debug("Trying to connect to rtde control interface")
            self.rtde_c = rtde_control.RTDEControlInterface(host)

            self.urmoco_out_queue.put(
                {"type": "info", "payload": {"status_text": "Configuring robot"}}
            )
            self.rtde_c.setTcp([0, 0, 0.1, 0, 0, 0])
            self.rtde_c.setPayload(
                float(self.config.get("robot.payload")), [0.0, 0.0, 0.0]
            )
            time.sleep(1)

            # Send the initial configuration to start correctly with the ghost
            joints = self.get_configuration()
            self.urmoco_out_queue.put({"type": "sync", "payload": {"joints": joints}})

            self.urmoco_out_queue.put({"type": "startup"})
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

    def move_to_configuration(self, q):
        speed = self.config.get("robot.joint_move_speed")
        acceleration = self.config.get("robot.joint_move_acceleration")
        self.rtde_c.moveJ(list(q), speed, acceleration, True)

    def get_configuration(self):
        return tuple(self.rtde_r.getActualQ())

    def get_joints_distance(self, target):
        q = self.get_configuration()
        dist = 0
        for i in range(6):
            dist += (target[i] - q[i]) ** 2
        return dist**0.5

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
        speed = self.config.get("robot.joint_stop_speeed")
        self.rtde_c.stopJ(speed)

    def unlock_protective_stop(self):
        self.dashboard_client.unlockProtectiveStop()

    def power_off(self):
        self.rtde_c.stopScript()
        self.dashboard_client.powerOff()

    def is_steady(self):
        return self.rtde_c.isSteady()
