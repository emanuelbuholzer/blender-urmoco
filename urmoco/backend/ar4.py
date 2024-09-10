import time

from serial import Serial
import numpy as np
import logging

logger = logging.getLogger(__name__)


class RobotClientAR4:
    """
  A = J1
  B = J2
  C = J3
  D = J4
  E = J5
  F = J6
  G = J7
  H = J8
  I = J9
  """

    def __init__(self, config, state, urmoco_out_queue):
        self.config = config
        self.urmoco_out_queue = urmoco_out_queue
        self.state = state
        self.comm = None

    def _verify_calibration_success(self, calibration_response: str):
        if calibration_response.startswith("E"):
            failing_joint = calibration_response[2]
            status_text = f"Calibration failed. Could not calibrate joint {failing_joint}"
            logger.error(status_text)
            self.urmoco_out_queue.put(
                {"type": "calibration_failure", "payload": {"status_text": status_text}}
            )

    def _calibrate(self):
        logger.info("Calibrating robot")
        self.urmoco_out_queue.put(
            {"type": "info", "payload": {"status_text": "Calibrating robot"}}
        )
        time.sleep(1.5)

        logger.info("Calibrating joint 1, 2 and 3")
        self.urmoco_out_queue.put(
            {"type": "info", "payload": {"status_text": "Calibrating joint 1, 2 and 3"}}
        )
        self.comm.write("LLA1B1C1D0E0F0G0H0I0J0K1L4M2N25O0P0Q0R0\n".encode())
        self._verify_calibration_success(self.comm.readline().decode())

        logger.info("Calibrating joint 6")
        self.urmoco_out_queue.put(
            {"type": "info", "payload": {"status_text": "Calibrating joint 6"}}
        )
        self.comm.write("LLA0B0C0D0E0F1G0H0I0J0K1L4M2N25O0P0Q0R0\n".encode())
        self._verify_calibration_success(self.comm.readline().decode())

        logger.info("Calibrating joint 4 and 5")
        self.urmoco_out_queue.put(
            {"type": "info", "payload": {"status_text": "Calibrating joint 4 and 5"}}
        )
        self.comm.write("LLA0B0C0D1E1F0G0H0I0J0K1L4M2N25O0P0Q0R0\n".encode())
        self._verify_calibration_success(self.comm.readline().decode())

    def calibrate(self):
        self._calibrate()
        logger.info("Successfully calibrated robot")
        self.urmoco_out_queue.put(
            {"type": "calibration_success"}
        )

    def connect(self):
        self.comm = Serial(self.config.get("ar4.port"))
        self.urmoco_out_queue.put(
            {"type": "info", "payload": {"status_text": "Connecting to the robot"}}
        )
        time.sleep(1)

        self._calibrate()
        self.urmoco_out_queue.put(
            {"type": "info", "payload": {"status_text": "Calibrated robot successfully"}}
        )

        # Send the initial configuration to start correctly with the ghost
        joints = self.get_configuration()
        self.urmoco_out_queue.put({"type": "sync", "payload": {"joints": joints}})

        self.urmoco_out_queue.put({"type": "startup"})
        return True

    def is_connected(self):
        return self.comm.is_open

    def disconnect(self):
        self.comm.write("CL\n".encode())
        self.comm.close()

    def power_off(self):
        pass

    def reconnect(self):
        if not self.is_connected():
            self.comm = Serial(self.config.get("ar4.port"))

    def move_to_configuration(self, q):
        q = np.rad2deg(q)
        # Speed, Acceleration, Deceleration, Ramp
        # RJA0B0C-89E0F0J70J80J90SP25Ac10Dc10Rm100WNLm00000
        # RJ
        # A = J1
        # B = J2
        # C = J3
        # D = J4
        # E = J5
        # F = J6
        # J7 = J7
        # J8 = J8
        # J9 = J9
        # S = Speed
        # A = Acceleration
        # Dc = Deceleration
        # Rm = Ramp
        # W = WristConStart = N
        # Lm = LoopModeStart = 000000
        speed = self.config.get("ar4.speed")
        acceleration = self.config.get("ar4.acceleration")
        deceleration = self.config.get("ar4.deceleration")
        ramp = self.config.get("ar4.ramp")
        cmd = f"RJA{str(q[0])}B{str(q[1])}C{str(q[2])}D{str(q[3])}E{str(q[4])}F{str(q[5])}J70J80J90Sp{str(speed)}Ac{str(acceleration)}Dc{str(deceleration)}Rm{str(ramp)}WNLm000000\n"
        self.comm.write(cmd.encode())

        res: str = self.comm.readline().decode()
        if res.startswith("ER"):
            # Kinematic error
            logger.error("Kinematic error")
            self.urmoco_out_queue.put({"type": "move_cancelled"})
            pass
        elif res.startswith("EL"):
            # Axis fault
            logger.error("Axis error")
            self.urmoco_out_queue.put({"type": "move_cancelled"})
            pass
        else:
            # HELP? Already robot pos?
            # End pos, goal reached... :-)
            logger.error("HELP?")
            pass
            self.state["frame"] = -1
            self.state["move"]["stopping"] = False
            self.state["move"]["scheduled"] = False
            self.state["move"]["active"] = False
            self.state["move"]["target_joints"] = None
            self.state["move"]["time_elapsed_seconds"] = 0
            joints = self.get_configuration()
            self.urmoco_out_queue.put({"type": "sync", "payload": {"joints": joints}})
            self.urmoco_out_queue.put({"type": "stop"})
            response = {
                "type": "move_success",
                "payload": {"frame": self.state["move"]["target_frame"]},
            }
            self.urmoco_out_queue.put(response)
            #ur_out_q.put(response)
            #df_out_q.put(response)
            # TODO
            # self.dfmoco_out_queue.put({"type": "stop_motor"})
            # self.dfmoco_out_queue.put({"type": "stop_all"})
            # self.dfmoco_out_queue.put(
            #     {"type": "set_frame", "payload": {"current_frame": state["frame"]}}
            # )

        print(res)

    def get_configuration(self):
        self.comm.write("RP\n".encode())
        res = self.comm.readline().decode()
        iA = res.index("A")
        iB = res.index("B")
        iC = res.index("C")
        iD = res.index("D")
        iE = res.index("E")
        iF = res.index("F")
        iG = res.index("G")
        iH = res.index("H")
        iI = res.index("I")
        iJ = res.index("J")
        iK = res.index("K")
        iL = res.index("L")
        iM = res.index("M")
        iN = res.index("N")
        iO = res.index("O")
        iP = res.index("P")
        iQ = res.index("Q")
        iR = res.index("R")
        iEnd = res.index("\r")

        valA = res[iA + 1: iB]
        valB = res[iB + 1: iC]
        valC = res[iC + 1: iD]
        valD = res[iD + 1: iE]
        valE = res[iE + 1: iF]
        valF = res[iF + 1: iG]
        valG = res[iG + 1: iH]
        valH = res[iH + 1: iI]
        valI = res[iI + 1: iJ]
        valJ = res[iJ + 1: iK]
        valK = res[iK + 1: iL]
        valL = res[iL + 1: iM]
        valM = res[iM + 1: iN]
        valN = res[iN + 1: iO]
        valO = res[iO + 1: iP]
        valP = res[iP + 1: iQ]
        valQ = res[iQ + 1: iR]
        valR = res[iR + 1: iEnd]

        q_deg = [float(valA), float(valB), float(valC), float(valD), float(valE), float(valF)]
        p_xyzuvw = [float(valG), float(valH), float(valI), float(valJ), float(valK), float(valL)]
        speed_violation = True if valM == "1" else False
        debug = valN
        flag = valO
        j7_pos = valP
        j8_pos = valQ
        j9_pos = valR
        logger.error(f"speed violation: {speed_violation}, flag: {flag}")
        return tuple(np.deg2rad(q_deg))

    ####
    ### TODO FROM HERE ON BELOW

    def get_mode(self):
        return 7

    def get_safety_mode(self):
        return 1

    def stop(self):
        pass

    def is_steady(self):
        return True
