from urmoco.config import Config

config = Config({})

initial_state = {
    'shooting': False,
    'frame': -1,
    'robot_mode': None,
    'safety_mode': None,
    'payload': config.get('robot.payload'),
    'move': {
        'active': False,
        'target_joints': None,
        'target_frame': None,
        'time_elapsed_seconds': 0.
    },
    'cycle': {
        'prev_duration_seconds': 0.
    }
}


def get_initial_state():
    return initial_state.copy()
