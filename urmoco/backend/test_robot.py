from unittest.mock import Mock, MagicMock

from urmoco.backend.robot import RobotClient


def test_connect(mocker):
    # Arrange
    config = Mock()
    config.get = Mock(return_value="some_host")

    dashboard_client_clazz: MagicMock = mocker.patch('dashboard_client.DashboardClient')
    dashboard_client = dashboard_client_clazz.return_value
    dashboard_client.connect = Mock()

    rtde_receive_clazz: MagicMock = mocker.patch('rtde_receive.RTDEReceiveInterface')

    rtde_control_clazz: MagicMock = mocker.patch('rtde_control.RTDEControlInterface')

    robot = RobotClient(config)
    robot.disconnect = Mock()

    # Act
    ret = robot.connect()

    # Assert
    config.get.assert_called_once_with('robot.host')
    dashboard_client_clazz.assert_called_once_with('some_host')
    dashboard_client.connect.assert_called_once()
    rtde_receive_clazz.assert_called_once_with('some_host')
    rtde_control_clazz.assert_called_once_with('some_host')
    robot.disconnect.assert_not_called()
    assert ret is True


def test_connect_fails(mocker):
    # Arrange
    config = Mock()
    config.get = Mock(return_value="some_host")

    dashboard_client_clazz: MagicMock = mocker.patch('dashboard_client.DashboardClient')
    dashboard_client = dashboard_client_clazz.return_value
    dashboard_client.connect = Mock()

    rtde_receive_clazz: MagicMock = mocker.patch('rtde_receive.RTDEReceiveInterface')

    rtde_control_clazz: MagicMock = mocker.patch('rtde_control.RTDEControlInterface')
    rtde_control_clazz.side_effect = Exception("some_error")

    robot = RobotClient(config)
    robot.disconnect = Mock()

    # Act
    ret = robot.connect()

    # Assert
    config.get.assert_called_once_with('robot.host')
    dashboard_client_clazz.assert_called_once_with('some_host')
    dashboard_client.connect.assert_called_once()
    rtde_receive_clazz.assert_called_once_with('some_host')
    rtde_control_clazz.assert_called_once_with('some_host')
    robot.disconnect.assert_called_once()
    assert ret is False


def test_is_connected():
    # Arrange
    config = Mock()
    robot = RobotClient(config)
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
    robot = RobotClient(config)
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
    robot = RobotClient(config)
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
    robot = RobotClient(config)
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
    robot = RobotClient(config)

    # Act
    robot.disconnect()

    # Assert
    assert robot.dashboard_client is None
    assert robot.rtde_r is None
    assert robot.rtde_c is None


def test_reconnect():
    # Arrange
    config = Mock()
    robot = RobotClient(config)

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
    robot = RobotClient(config)

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
