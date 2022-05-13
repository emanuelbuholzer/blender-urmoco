import socket
import logging

logger = logging.getLogger(__name__)

RESPONSE_CONNECTED = "Connected: Universal Robots Dashboard Server"
RESPONSE_POWER_ON = "Powering on"
RESPONSE_BRAKE_RELEASE = "Brake releasing"
RESPONSE_UNLOCK = "Protective stop releasing"
RESPONSE_POPUP = "closing popup"
RESPONSE_SAFETY_POPUP = "closing safety popup"
RESPONSE_POWER_OFF = "Powering off"
RESPONSE_SHUTDOWN = "Shutting down"


class DashboardClient:

    def __init__(self, config):
        self.config = config

    def _send_request(self, request):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((self.config.get('robot.host'), 29999))
        response = s.recv(4096).decode().rstrip()
        if response != RESPONSE_CONNECTED:
            raise Exception(f"Cannot connect to the dashboard server, received: {response}")
        s.sendall(f"{request}\n".encode())
        return s.recv(4096).decode().rstrip()

    def get_mode(self):
        return self._send_request("robotmode").removeprefix('Robotmode: ').strip()

    def get_safety_mode(self):
        return self._send_request("safetymode").removeprefix('Safetymode: ').strip()

    def power_on(self):
        response = self._send_request("power on")
        logger.info(response)
        if response != RESPONSE_POWER_ON:
            raise Exception(f"Cannot power on the robot, received: {response}")

    def release_brakes(self):
        response = self._send_request("brake release")
        if response != RESPONSE_BRAKE_RELEASE:
            raise Exception(f"Cannot release the brakes, received: {response}")

    def unlock_protective_stop(self):
        response = self._send_request("unlock protective stop")
        if response != RESPONSE_UNLOCK:
            raise Exception(f"Cannot unlock the protective stop, received: {response}")

    def close_popup(self):
        response = self._send_request("close popup")
        if response != RESPONSE_POPUP:
            raise Exception(f"Cannot close the popup, received: {response}")

    def close_safety_popup(self):
        response = self._send_request("close safety popup")
        if response != RESPONSE_SAFETY_POPUP:
            raise Exception(f"Cannot close the safety popup, received: {response}")

    def power_off(self):
        response = self._send_request("power off")
        if response != RESPONSE_POWER_OFF:
            raise Exception(f"Cannot power off the robot, received: {response}")

    def shutdown(self):
        response = self._send_request("shutdown")
        if response != RESPONSE_SHUTDOWN:
            raise Exception(f"Cannot shutdown the robot, received: {response}")

    def is_program_running(self):
        response = self._send_request("running")
        if response == "Program running: True":
            return True
        elif response == "Program running: False":
            return False
        else:
            raise Exception(f"Cannot check if program is running, received: {response}")
