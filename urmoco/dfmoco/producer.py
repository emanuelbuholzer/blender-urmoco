import asyncio
import logging
import queue
from asyncio import StreamWriter
from multiprocessing import Queue

from urmoco.dfmoco.state import DFMocoState

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)


async def stop_all(state: DFMocoState, _response_payload, writer: StreamWriter):
    state.is_moving = False
    state.current_frame = -1
    writer.write(b"sa\r\n")
    await writer.drain()


async def stop_motor(state: DFMocoState, _response_payload, writer: StreamWriter):
    state.is_moving = False
    state.current_frame = -1
    writer.write(b"sm 1\r\n")
    await writer.drain()


async def set_frame(state: DFMocoState, response_payload, writer: StreamWriter):
    state.current_frame = response_payload["current_frame"]
    message = f"mp 1 {state.current_frame}\r\n"
    writer.write(bytes(message, encoding="ascii"))
    await writer.drain()


async def move_to_frame(state: DFMocoState, response_payload, writer: StreamWriter):
    state.is_moving = True
    state.current_frame = -1
    response = f'mm 1 {response_payload["target_frame"]}\r\n'
    writer.write(bytes(response, encoding="ascii"))
    await writer.drain()


async def is_moving(state: DFMocoState, response_payload, writer: StreamWriter):
    state.is_moving = response_payload["is_moving"]

    response = f"ms {1 if state.is_moving else 0}\r\n"
    writer.write(bytes(response, encoding="ascii"))
    await writer.drain()


async def handle_move_timeout(
    state: DFMocoState, _response_payload, writer: StreamWriter
):
    await set_frame(state, {"current_frame": -1}, writer)
    await is_moving(state, {"is_moving": False}, writer)


async def handle_move_success(
    state: DFMocoState, response_payload, writer: StreamWriter
):
    # It is important, that the current frame gets set and sent before we send Dragonframe the information,
    # that we're no longer moving. Dragonframe can and will trigger another move once it receives the information,
    # that we're no longer moving, even though we might have not arrived there or didn't even start!
    await set_frame(state, {"current_frame": response_payload["frame"]}, writer)
    await is_moving(state, {"is_moving": False}, writer)


response_handlers = {
    "stop_all": stop_all,
    "stop_motor": stop_motor,
    "set_frame": set_frame,
    "move_to_frame": move_to_frame,
    "is_moving": is_moving,
    "move_timeout": handle_move_timeout,
    "move_success": handle_move_success,
}


async def run_cycle(state: DFMocoState, out_queue: Queue, writer: StreamWriter):
    try:
        response = out_queue.get_nowait()
        response_type = response["type"]
        response_payload = response.get("payload", False) or {}

        handler = response_handlers.get(response_type)
        if handler is None:
            raise NotImplementedError("No handler for response type %s.", response_type)

        await handler(state, response_payload, writer)

    except queue.Empty:
        # Apparently heartbeats are not necessary, if you end up again here, then that was a lie x)
        await asyncio.sleep(0.01)


async def run(state: DFMocoState, out_queue: Queue, writer: StreamWriter):
    while True:
        await run_cycle(state, out_queue, writer)
