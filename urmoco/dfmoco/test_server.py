from asyncio import StreamWriter
from multiprocessing import Queue
from unittest.mock import AsyncMock, Mock, call

import pytest


@pytest.fixture
def out_queue() -> Queue:
    return Mock()


@pytest.fixture
def in_queue() -> Queue:
    return Mock()


@pytest.fixture
def writer() -> StreamWriter:
    writer = Mock()
    writer.drain = AsyncMock()
    return writer


@pytest.fixture
def reader() -> StreamWriter:
    writer = Mock()
    writer.drain = AsyncMock()
    return writer


def test_clear_queue():
    # Arrange
    q = Queue()
    q.put({"type": "some_message"})
    q.put({"type": "some_other_message"})
    q.put({"type": "some_some_message"})

    from urmoco.dfmoco import server

    # Act
    server.clear_queue(q)

    # Assert
    assert q.empty()


@pytest.mark.asyncio
async def test_clear_queues_on_new_session(mocker, in_queue, out_queue, reader, writer):
    # Arrange
    create_task = mocker.patch("asyncio.create_task")
    gather = mocker.patch("asyncio.gather", new_callable=AsyncMock)

    from urmoco.dfmoco import server

    connection_handler = server.get_connection_handler(in_queue, out_queue)
    server.clear_queue = Mock()

    # Act
    await connection_handler(reader, writer)

    # Assert
    server.clear_queue.assert_has_calls(
        [call(in_queue), call(out_queue)], any_order=True
    )

    writer.write.assert_has_calls([call(b"hi 1 1 1.2.5\r\n"), call(b"mp 1 -1\r\n")])
    writer.drain.assert_called_once()

    create_task.assert_called()
    gather.assert_called_once()
