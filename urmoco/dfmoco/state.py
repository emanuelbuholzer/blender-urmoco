from dataclasses import dataclass


@dataclass
class DFMocoState:
    is_moving: bool
    current_frame: int
