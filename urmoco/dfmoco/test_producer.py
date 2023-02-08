import queue
from asyncio import StreamWriter
from multiprocessing import Queue
from unittest.mock import AsyncMock, Mock, call

import pytest

from urmoco.config import Config
from urmoco.dfmoco import producer
from urmoco.dfmoco.state import DFMocoState


@pytest.fixture
def config() -> Config:
    return Mock()


@pytest.fixture
def out_queue() -> Queue:
    return Mock()


@pytest.fixture
def writer() -> StreamWriter:
    writer = Mock()
    writer.drain = AsyncMock()
    return writer


@pytest.mark.asyncio
async def test_send_motor_position_heartbeat(config, out_queue, writer):
    # Arrange
    state = DFMocoState(current_frame=42, is_moving=False)
    config.get.return_value = 12
    out_queue.get_nowait.side_effect = queue.Empty()

    # Act
    await producer.run_cycle(config, state, out_queue, writer)

    # Assert
    writer.write.assert_called_once_with(b"mp 1 42\r\n")
    writer.drain.assert_called_once()


@pytest.mark.asyncio
async def test_send_motor_status_heartbeat(config, out_queue, writer):
    # Arrange
    state = DFMocoState(current_frame=-1, is_moving=True)
    config.get.return_value = 12
    out_queue.get_nowait.side_effect = queue.Empty()

    # Act
    await producer.run_cycle(config, state, out_queue, writer)

    # Assert
    writer.write.assert_called_once_with(b"ms 1\r\n")
    writer.drain.assert_called_once()


@pytest.mark.asyncio
async def test_handle_unknown_message(config, out_queue, writer):
    # Arrange
    out_queue.get_nowait.return_value = {"type": "blablabla"}

    # Act & assert
    with pytest.raises(NotImplementedError):
        await producer.run(
            config, {"is_moving": False, "current_frame": -1}, out_queue, writer
        )


@pytest.mark.asyncio
async def test_stop_all(writer):
    # Arrange
    state = DFMocoState(current_frame=42, is_moving=True)

    # Act
    await producer.stop_all(state, None, writer)

    # Assert
    writer.write.assert_called_once_with(b"sa\r\n")
    writer.drain.assert_called_once()
    assert not state.is_moving
    assert state.current_frame == -1


@pytest.mark.asyncio
async def test_stop_motor(writer):
    # Arrange
    state = DFMocoState(current_frame=24, is_moving=True)

    # Act
    await producer.stop_motor(state, None, writer)

    # Assert
    writer.write.assert_called_once_with(b"sm 1\r\n")
    writer.drain.assert_called_once()
    assert not state.is_moving
    assert state.current_frame == -1


@pytest.mark.asyncio
async def test_set_frame(writer):
    # Arrange
    state = DFMocoState(current_frame=24, is_moving=True)

    # Act
    await producer.set_frame(state, {"current_frame": 42}, writer)

    # Assert
    writer.write.assert_called_once_with(b"mp 1 42\r\n")
    writer.drain.assert_called_once()
    assert state.is_moving
    assert state.current_frame == 42


@pytest.mark.asyncio
async def test_move_to_frame(writer):
    # Arrange
    state = DFMocoState(current_frame=1, is_moving=False)

    # Act
    await producer.move_to_frame(state, {"target_frame": 42}, writer)

    # Assert
    writer.write.assert_called_once_with(b"mm 1 42\r\n")
    writer.drain.assert_called_once()
    assert state.is_moving
    assert state.current_frame == -1


@pytest.mark.asyncio
async def test_is_moving_starting(writer):
    # Arrange
    state = DFMocoState(current_frame=12, is_moving=False)

    # Act
    await producer.is_moving(state, {"is_moving": True}, writer)

    # Assert
    writer.write.assert_called_once_with(b"ms 1\r\n")
    writer.drain.assert_called_once()
    assert state.is_moving
    assert state.current_frame == -1


@pytest.mark.asyncio
async def test_is_moving_stopping(writer):
    # Arrange
    state = DFMocoState(current_frame=-1, is_moving=True)

    # Act
    await producer.is_moving(state, {"is_moving": False}, writer)

    # Assert
    writer.write.assert_called_once_with(b"ms 0\r\n")
    writer.drain.assert_called_once()
    assert not state.is_moving
    assert state.current_frame == -1


@pytest.mark.asyncio
async def test_is_moving_already_stopped(writer):
    # Arrange
    state = DFMocoState(current_frame=12, is_moving=False)

    # Act
    await producer.is_moving(state, {"is_moving": False}, writer)

    # Assert
    writer.write.assert_called_once_with(b"ms 0\r\n")
    writer.drain.assert_called_once()
    assert not state.is_moving
    assert state.current_frame == 12


@pytest.mark.asyncio
async def test_handle_move_timeout(writer):
    # Arrange
    state = DFMocoState(current_frame=12, is_moving=True)

    # Act
    await producer.handle_move_timeout(state, None, writer)

    # Assert
    writer.write.assert_has_calls([call(b"ms 0\r\n"), call(b"mp 1 -1\r\n")])
    writer.drain.assert_called()
    assert not state.is_moving
    assert state.current_frame == -1


@pytest.mark.asyncio
async def test_handle_move_success(writer):
    # Arrange
    state = DFMocoState(current_frame=-1, is_moving=True)

    # Act
    await producer.handle_move_success(state, {"frame": 42}, writer)

    # Assert
    writer.write.assert_has_calls([call(b"ms 0\r\n"), call(b"mp 1 42\r\n")])
    writer.drain.assert_called()
    assert not state.is_moving
    assert state.current_frame == 42
