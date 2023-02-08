from multiprocessing import Queue
from unittest.mock import MagicMock, Mock, call

from dashboard_client import DashboardClient
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface

from urmoco.backend.robot import RobotClient
from urmoco.config import Config


def test_connect(mocker):
    # Arrange
    config: Config = Mock()
    config.get.side_effect = ["some_host", 2]

    dashboard_client_clazz: Mock = mocker.patch("dashboard_client.DashboardClient")
    dashboard_client: DashboardClient = dashboard_client_clazz.return_value
    dashboard_client.robotmode.return_value = "Robotmode: POWER_OFF"

    rtde_receive_clazz: Mock = mocker.patch("rtde_receive.RTDEReceiveInterface")
    rtde_r: RTDEReceiveInterface = rtde_receive_clazz.return_value
    rtde_r.getActualQ.return_value = [0.0, 0.2, 0.4, 1.0, 1.2, 1.4]

    rtde_control_clazz: Mock = mocker.patch("rtde_control.RTDEControlInterface")
    rtde_c: RTDEControlInterface = rtde_control_clazz.return_value

    urmoco_out_queue: Queue = Mock()

    robot = RobotClient(config, Mock(), urmoco_out_queue)
    robot.disconnect = Mock()

    # Act
    ret = robot.connect()

    # Assert
    config.get.assert_has_calls([call("robot.host"), call("robot.payload")])

    dashboard_client_clazz.assert_called_once_with("some_host")
    dashboard_client.connect.assert_called_once()
    dashboard_client.robotmode.assert_called_once()
    dashboard_client.closePopup.assert_called_once()
    dashboard_client.closeSafetyPopup.assert_called_once()
    dashboard_client.powerOn.assert_called_once()
    dashboard_client.brakeRelease()

    rtde_receive_clazz.assert_called_once_with("some_host")
    rtde_control_clazz.assert_called_once_with("some_host")
    rtde_c.setTcp.assert_called_once()
    rtde_c.setPayload.assert_called_once_with(2, [0.0, 0.0, 0.0])
    rtde_r.getActualQ.assert_called_once()

    assert (
        call({"type": "sync", "payload": {"joints": (0.0, 0.2, 0.4, 1.0, 1.2, 1.4)}})
        in urmoco_out_queue.put.mock_calls
    )
    assert call({"type": "startup"}) in urmoco_out_queue.put.mock_calls

    assert ret is True

    robot.disconnect.assert_not_called()


def test_connect_fails(mocker):
    # Arrange
    config: Config = Mock()
    dashboard_client_clazz: Mock = mocker.patch("dashboard_client.DashboardClient")
    urmoco_out_queue: Queue = Mock()
    robot = RobotClient(config, Mock(), urmoco_out_queue)
    robot.disconnect = Mock()

    # Act
    ret = robot.connect()

    # Assert
    config.get.assert_called_once()
    dashboard_client_clazz.assert_called_once()
    robot.disconnect.assert_called_once()
    assert ret is False
    assert robot.dashboard_client is None
    assert robot.rtde_r is None
    assert robot.rtde_c is None


def test_is_connected():
    # Arrange
    config = Mock()
    state = Mock()
    urmoco_out_queue = Mock()

    robot = RobotClient(config, state, urmoco_out_queue)
    robot.dashboard_client = Mock()
    robot.dashboard_client.isConnected = Mock(return_value=True)
    robot.rtde_r = Mock()
    robot.rtde_r.isConnected = Mock(return_value=True)
    robot.rtde_c = Mock()
    robot.rtde_c.isConnected = Mock(return_value=True)

    # Act
    ret = robot.is_connected()

    # Assert
    robot.dashboard_client.isConnected.assert_called_once()
    robot.rtde_r.isConnected.assert_called_once()
    robot.rtde_c.isConnected.assert_called_once()
    assert ret is True


def test_is_connected_but_it_is_not1():
    # Arrange
    config = Mock()
    state = Mock()
    urmoco_out_queue = Mock()

    robot = RobotClient(config, state, urmoco_out_queue)
    robot.dashboard_client = Mock()
    robot.dashboard_client.isConnected = Mock(return_value=True)
    robot.rtde_r = Mock()
    robot.rtde_r.isConnected = Mock(return_value=True)
    robot.rtde_c = Mock()
    robot.rtde_c.isConnected = Mock(return_value=False)

    # Act
    ret = robot.is_connected()

    # Assert
    robot.dashboard_client.isConnected.assert_called_once()
    robot.rtde_r.isConnected.assert_called_once()
    robot.rtde_c.isConnected.assert_called_once()
    assert ret is False


def test_is_connected_but_it_is_not2():
    # Arrange
    config = Mock()
    state = Mock()
    urmoco_out_queue = Mock()

    robot = RobotClient(config, state, urmoco_out_queue)
    robot.dashboard_client = Mock()
    robot.dashboard_client.isConnected = Mock(return_value=False)
    robot.rtde_r = Mock()
    robot.rtde_r.isConnected = Mock(return_value=False)
    robot.rtde_c = Mock()
    robot.rtde_c.isConnected = Mock(return_value=False)

    # Act
    ret = robot.is_connected()

    # Assert
    robot.dashboard_client.isConnected.assert_called_once()
    robot.rtde_r.isConnected.assert_called_once()
    robot.rtde_c.isConnected.assert_called_once()
    assert ret is False


def test_disconnect():
    # Arrange
    config = Mock()
    state = Mock()
    urmoco_out_queue = Mock()

    robot = RobotClient(config, state, urmoco_out_queue)
    dashboard_client = Mock()
    robot.dashboard_client = dashboard_client
    robot.dashboard_client.disconnect = Mock()
    rtde_r = Mock()
    robot.rtde_r = rtde_r
    robot.rtde_r.disconnect = Mock()
    rtde_c = Mock()
    robot.rtde_c = rtde_c
    robot.rtde_c.disconnect = Mock()

    # Act
    robot.disconnect()

    # Assert
    assert robot.dashboard_client is None
    dashboard_client.disconnect.assert_called_once()
    assert robot.rtde_r is None
    rtde_r.disconnect.assert_called_once()
    assert robot.rtde_c is None
    rtde_c.disconnect.assert_called_once()


def test_disconnect_noop():
    # Arrange
    config = Mock()
    state = Mock()
    urmoco_out_queue = Mock()

    robot = RobotClient(config, state, urmoco_out_queue)

    # Act
    robot.disconnect()

    # Assert
    assert robot.dashboard_client is None
    assert robot.rtde_r is None
    assert robot.rtde_c is None


def test_reconnect():
    # Arrange
    config = Mock()
    state = Mock()
    urmoco_out_queue = Mock()

    robot = RobotClient(config, state, urmoco_out_queue)

    robot.dashboard_client = Mock()
    robot.dashboard_client.isConnected = Mock(return_value=True)
    robot.dashboard_client.connect = Mock()
    robot.rtde_r = Mock()
    robot.rtde_r.isConnected = Mock(return_value=True)
    robot.rtde_r.reconnect = Mock()
    robot.rtde_c = Mock()
    robot.rtde_c.isConnected = Mock(return_value=True)
    robot.rtde_c.reconnect = Mock()

    # Act
    robot.reconnect()

    # Assert
    robot.dashboard_client.isConnected.assert_called_once()
    robot.dashboard_client.connect.assert_not_called()
    robot.rtde_r.isConnected.assert_called_once()
    robot.rtde_r.reconnect.assert_not_called()
    robot.rtde_c.isConnected.assert_called_once()
    robot.rtde_c.reconnect.assert_not_called()


def test_reconnect_with_actual_reconnects():
    # Arrange
    config = Mock()
    state = Mock()
    urmoco_out_queue = Mock()

    robot = RobotClient(config, state, urmoco_out_queue)

    robot.dashboard_client = Mock()
    robot.dashboard_client.isConnected = Mock(return_value=False)
    robot.dashboard_client.connect = Mock()
    robot.rtde_r = Mock()
    robot.rtde_r.isConnected = Mock(return_value=False)
    robot.rtde_r.reconnect = Mock()
    robot.rtde_c = Mock()
    robot.rtde_c.isConnected = Mock(return_value=False)
    robot.rtde_c.reconnect = Mock()

    # Act
    robot.reconnect()

    # Assert
    robot.dashboard_client.isConnected.assert_called_once()
    robot.dashboard_client.connect.assert_called_once()
    robot.rtde_r.isConnected.assert_called_once()
    robot.rtde_r.reconnect.assert_called_once()
    robot.rtde_c.isConnected.assert_called_once()
    robot.rtde_c.reconnect.assert_called_once()
