from enum import Enum


class Mode(Enum):
    LOCKED = "LOCKED"
    OFF = "OFF"
    ON = "ON"
    DISCONNECTED = "DISCONNECTED"
    FREEDRIVE = "FREEDRIVE"
    SHOOTING = "SHOOTING"
    AWAIT_RESPONSE = "AWAIT_RESPONSE"
    ERROR = "ERROR"
