import multiprocessing
import socket
import time

import pytest

from .config import Config
from .dfmoco.proc import run as run_dfmoco

config = Config(
    {
        "dfmoco": {
            "host": "localhost",
            "port": 25555,
            "producer_interval_seconds": 1,
        }
    }
)


@pytest.fixture
def dfmoco_server():
    return DFMocoProcess()


class DFMocoProcess:
    def __init__(self):
        mp_context = multiprocessing.get_context("fork")
        self.dfmoco_in_queue = mp_context.Queue()
        self.dfmoco_out_queue = mp_context.Queue()
        self.dfmoco_proc = mp_context.Process(
            target=run_dfmoco,
            name="DFMoco process",
            args=(self.dfmoco_in_queue, self.dfmoco_out_queue),
            daemon=True,
        )
        self.dfmoco_proc.start()
        time.sleep(0.1)

    def stop(self):
        self.dfmoco_proc.kill()
        time.sleep(0.1)


@pytest.fixture
def dfmoco_client(dfmoco_server):
    return DFMocoClient(dfmoco_server)


class DFMocoClient:
    def __init__(self, dfmoco_server):
        self.dfmoco_in_queue = dfmoco_server.dfmoco_in_queue
        self.dfmoco_out_queue = dfmoco_server.dfmoco_out_queue
        self.dfmoco_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.dfmoco_server = dfmoco_server
        self.f = self.dfmoco_socket.makefile()

    def connect(self):
        self.dfmoco_socket.connect(
            (config.get("dfmoco.host"), config.get("dfmoco.port"))
        )

    def send(self, message):
        self.dfmoco_socket.sendall(bytes(message, encoding="ascii"))

    def readline(self):
        return self.f.readline()

    def trigger_event(self, event):
        self.dfmoco_out_queue.put(event)

    def expect_event(self, event):
        assert self.dfmoco_in_queue.get() == event

    def stop(self):
        self.dfmoco_socket.close()
        self.dfmoco_server.stop()


@pytest.mark.timeout(1)
def test_receive_hi(dfmoco_client):
    dfmoco_client.connect()
    dfmoco_client.send("hi\r\n")
    assert dfmoco_client.readline() == "hi 1 1 1.2.5\n"
    dfmoco_client.stop()


@pytest.mark.timeout(1)
def test_receive_heartbeat(dfmoco_client):
    dfmoco_client.connect()
    assert dfmoco_client.readline() == "hi 1 1 1.2.5\n"
    assert dfmoco_client.readline() == "mp 1 -1\n"
    dfmoco_client.stop()


@pytest.mark.timeout(2)
def test_is_not_moving(dfmoco_client):
    dfmoco_client.connect()
    assert dfmoco_client.readline() == "hi 1 1 1.2.5\n"
    assert dfmoco_client.readline() == "mp 1 -1\n"

    dfmoco_client.send("ms\r\n")
    dfmoco_client.expect_event({"type": "is_moving"})

    dfmoco_client.trigger_event({"type": "is_moving", "payload": {"is_moving": False}})
    assert dfmoco_client.readline() == "ms 0\n"
    dfmoco_client.stop()


@pytest.mark.timeout(2)
def test_is_moving(dfmoco_client):
    dfmoco_client.connect()

    assert dfmoco_client.readline() == "hi 1 1 1.2.5\n"
    assert dfmoco_client.readline() == "mp 1 -1\n"

    dfmoco_client.send("ms\r\n")
    dfmoco_client.expect_event({"type": "is_moving"})

    dfmoco_client.trigger_event({"type": "is_moving", "payload": {"is_moving": True}})
    dfmoco_client.send("ms\r\n")
    assert dfmoco_client.readline() == "ms 1\n"
    dfmoco_client.stop()


@pytest.mark.timeout(2)
def test_stop_motor(dfmoco_client):
    dfmoco_client.connect()

    assert dfmoco_client.readline() == "hi 1 1 1.2.5\n"
    assert dfmoco_client.readline() == "mp 1 -1\n"

    dfmoco_client.send("sm 1\r\n")
    dfmoco_client.expect_event({"type": "stop_motor"})

    dfmoco_client.trigger_event({"type": "stop_motor"})
    assert dfmoco_client.readline() == "sm 1\n"
    dfmoco_client.stop()


@pytest.mark.timeout(2)
def test_stop_all(dfmoco_client):
    dfmoco_client.connect()

    assert dfmoco_client.readline() == "hi 1 1 1.2.5\n"
    assert dfmoco_client.readline() == "mp 1 -1\n"

    dfmoco_client.send("sa\r\n")
    dfmoco_client.expect_event({"type": "stop_all"})

    dfmoco_client.trigger_event({"type": "stop_all"})
    assert dfmoco_client.readline() == "sa\n"
    dfmoco_client.stop()


@pytest.mark.timeout(10)
def test_move_to_frame(dfmoco_client):
    dfmoco_client.connect()

    assert dfmoco_client.readline() == "hi 1 1 1.2.5\n"
    assert dfmoco_client.readline() == "mp 1 -1\n"

    dfmoco_client.send("mm 1 15\r\n")
    dfmoco_client.expect_event(
        {"type": "move_to_frame", "payload": {"target_frame": 15}}
    )

    dfmoco_client.trigger_event(
        {"type": "move_to_frame", "payload": {"target_frame": 15}}
    )
    assert dfmoco_client.readline() == "mm 1 15\n"

    dfmoco_client.send("ms\r\n")
    dfmoco_client.expect_event({"type": "is_moving"})

    dfmoco_client.trigger_event({"type": "is_moving", "payload": {"is_moving": True}})
    dfmoco_client.send("ms\r\n")
    assert dfmoco_client.readline() == "ms 1\n"

    dfmoco_client.trigger_event({"type": "set_frame", "payload": {"current_frame": 15}})
    assert dfmoco_client.readline() == "mp 1 15\n"

    dfmoco_client.stop()
