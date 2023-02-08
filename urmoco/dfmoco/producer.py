import asyncio
import logging
import queue

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)


async def stop_all(state, _response_payload, writer):
    state["is_moving"] = False
    response = f"sa\r\n"
    writer.write(bytes(response, encoding="ascii"))
    await writer.drain()


async def stop_motor(state, _response_payload, writer):
    state["is_moving"] = False
    response = "sm 1\r\n"
    writer.write(bytes(response, encoding="ascii"))
    await writer.drain()


async def set_frame(state, response_payload, writer):
    state["current_frame"] = response_payload["current_frame"]
    heartbeat_message = f"mp {1} {state['current_frame']}\r\n"
    writer.write(bytes(heartbeat_message, encoding="ascii"))
    await writer.drain()


async def move_to_frame(state, response_payload, writer):
    state["is_moving"] = True
    response = f'mm 1 {response_payload["target_frame"]}\r\n'
    writer.write(bytes(response, encoding="ascii"))
    await writer.drain()


async def is_moving(state, response_payload, writer):
    state["is_moving"] = response_payload["is_moving"]

    response = f'ms {1 if state["is_moving"] else 0}\r\n'
    writer.write(bytes(response, encoding="ascii"))
    await writer.drain()


async def handle_move_timeout(state, _response_payload, writer):
    await is_moving(state, {"is_moving": False}, writer)
    await set_frame(state, {"current_frame": -1}, writer)


async def handle_move_success(state, response_payload, writer):
    await is_moving(state, {"is_moving": False}, writer)
    await set_frame(state, {"current_frame": response_payload["frame"]}, writer)


response_handlers = {
    "stop_all": stop_all,
    "stop_motor": stop_motor,
    "set_frame": set_frame,
    "move_to_frame": move_to_frame,
    "is_moving": is_moving,
    "move_timeout": handle_move_timeout,
    "move_success": handle_move_success,
}


async def run(config, state, out_queue, writer):
    while True:
        try:
            response = out_queue.get_nowait()
            response_type = response["type"]
            response_payload = response.get("payload", False) or {}

            handler = response_handlers.get(response_type)
            if handler is None:
                logger.error(
                    "No handler for response type %s. Ignoring.", response_type
                )
                continue

            await handler(state, response_payload, writer)

        except queue.Empty:
            heartbeat_message = f"mp {1} {state['current_frame']}\r\n"
            writer.write(bytes(heartbeat_message, encoding="ascii"))
            await writer.drain()
            await asyncio.sleep(config.get("dfmoco.producer_interval_seconds"))
