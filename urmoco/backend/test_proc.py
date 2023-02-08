from unittest.mock import MagicMock, Mock


def test_run(mocker):
    # Arrange
    config = Mock()
    urmoco_in_queue = Mock()
    dfmoco_in_queue = Mock()
    urmoco_out_queue = Mock()
    urmoco_out_queue.put = Mock()
    dfmoco_out_queue = Mock()

    robot_clazz: MagicMock = mocker.patch("urmoco.backend.robot.RobotClient")
    robot = robot_clazz.return_value
    robot.connect = Mock()
    robot.is_connected = Mock(return_value=True)
    robot.disconnect = Mock()

    get_initial_state: MagicMock = mocker.patch(
        "urmoco.backend.state.get_initial_state"
    )

    run_cycle: MagicMock = mocker.patch("urmoco.backend.cycle.run_cycle")
    run_cycle.side_effect = Exception("just_to_end_this")

    # Act
    from urmoco.backend.proc import run

    run(config, urmoco_in_queue, dfmoco_in_queue, urmoco_out_queue, dfmoco_out_queue)

    # Assert
    robot.connect.assert_called_once()
    robot.is_connected.assert_called_once()
    get_initial_state.assert_called_once()
    run_cycle.assert_called_once()
    urmoco_out_queue.put.assert_called_once()
    robot.disconnect.assert_called_once()
