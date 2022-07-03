from unittest.mock import Mock
from urmoco.config import Config

from urmoco.backend.move import handle_move
from urmoco.backend.state import get_initial_state


def test_move_reaching_target():
    # Arrange
    config = Config({})
    ur_out_q = Mock()
    df_out_q = Mock()

    state = get_initial_state()
    state["move"]["target_frame"] = 12

    robot = Mock()
    robot.get_joints_distance = Mock(return_value=0.000001)

    # Act
    handle_move(config, state, robot, ur_out_q, df_out_q)

    # Assert
    assert not state["move"]["active"]
    assert state["move"]["target_joints"] is None
    assert state["move"]["time_elapsed_seconds"] == 0

    res = {'type': 'move_success', 'payload': {'frame': 12}}
    ur_out_q.put.assert_called_once_with(res)
    df_out_q.put.assert_called_once_with(res)


def test_record_move_for_timeouts_before_reaching_target():
    # Arrange
    config = Config({})
    state = get_initial_state()
    ur_out_q = Mock()
    df_out_q = Mock()

    robot = Mock()
    robot.get_joints_distance = Mock(return_value=1)

    durations = [0.012, 0.043, 0.091]

    # Act
    for duration in durations:
        state['cycle']['prev_duration_seconds'] = duration
        handle_move(config, state, robot, ur_out_q, df_out_q)

    # Assert
    robot.stop.assert_not_called()
    ur_out_q.put.assert_not_called()
    df_out_q.put.assert_not_called()

    assert state['move']['time_elapsed_seconds'] == sum(durations)


def test_move_timeout_will_stop():
    # Arrange
    config = Config({
        'robot': {'move_timeout_seconds': 5}
    })
    state = get_initial_state()
    ur_out_q = Mock()
    df_out_q = Mock()

    robot = Mock()
    robot.get_joints_distance = Mock(return_value=1)

    durations = [2.012, 1.043, 2.091]

    # Act
    for duration in durations:
        state['cycle']['prev_duration_seconds'] = duration
        handle_move(config, state, robot, ur_out_q, df_out_q)

    # Assert
    robot.stop.assert_called_once()

    assert not state["move"]["active"]
    assert state["move"]["target_joints"] is None
    assert state["move"]["time_elapsed_seconds"] == 0

    ur_out_q.put.assert_called_once_with({'type': 'move_timeout'})
    df_out_q.put.assert_called_once_with({'type': 'move_timeout'})
