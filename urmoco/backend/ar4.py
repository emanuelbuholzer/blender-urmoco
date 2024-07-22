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

    def _calibrate(self):

        self.urmoco_out_queue.put(
            {"type": "info", "payload": {"status_text": "Calibrating robot"}}
        )
        # [0.0, -1.0, -4.0, -2.0, 65.0, 0.0]
        self.comm.write("LLA1B1C1D0E0F0G0H0I0J0K1L4M2N25O0P0Q0R0\n".encode())
        print(self.comm.readline().decode())
        self.comm.write("LLA0B0C0D0E0F1G0H0I0J0K1L4M2N25O0P0Q0R0\n".encode())
        print(self.comm.readline().decode())
        self.comm.write("LLA0B0C0D1E1F0G0H0I0J0K1L4M2N25O0P0Q0R0\n".encode())
        print(self.comm.readline().decode())
        # J to R probably offsets

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

    def reconnect(self):
        if not self.is_connected():
            self.comm = Serial(self.config.get("ar4.port"))

    def move_to_configuration(self, q):
        logger.error("MOVE TO CONFIGURATION - VERIFY DEG/RAD")
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
        print(cmd)
        self.comm.write(cmd.encode())
        print(self.comm.readline().decode())

    def get_configuration(self):
        logger.info("Requesting position, potentially something is alarming")
        self.comm.write("RP\n".encode())
        # A45.000B0.000C-88.992D0.000E0.000F0.000G48.676H48.676I738.359J45.000K1.008L0.000P0.000Q0.00R0.00\r\n"
        res = self.comm.readline().decode()
        print(res)
        logger.info(f"Request position result: {repr(res)}")

        iA = res.index("A")
        print(type(iA))
        iB = res.index("B")
        print(type(iB))
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
        print("Q")
        print(q_deg)
        logger.error("GET_CONFIGURATION - VERIFY DEG/RAD")
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





