import logging
from asyncio import StreamReader
from multiprocessing import Queue

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


async def stop_all(_request_type: bytes, _request_args: [bytes], in_queue: Queue):
    in_queue.put({"type": "stop_all"})


async def stop_motor(_request_type: bytes, _request_args: [bytes], in_queue: Queue):
    in_queue.put({"type": "stop_motor"})


async def move_to_frame(_request_type: bytes, request_args: [bytes], in_queue: Queue):
    target_frame = int(request_args[1])
    in_queue.put(
        {
            "type": "move_to_frame",
            "payload": {
                "target_frame": target_frame,
            },
        }
    )


async def get_motor_status(
    _request_type: bytes, _request_args: [bytes], in_queue: Queue
):
    in_queue.put({"type": "is_moving"})


async def no_operation(request_type: bytes, _request_args: [bytes], _in_queue: Queue):
    logger.debug(f"Ignoring operation for request type {request_type}")


async def unsupported_operation(
    request_type: bytes, _request_args: [bytes], _in_queue: Queue
):
    logger.error(f"Unsupported operation {request_type} requested. Ignoring.")


request_handlers = {
    b"hi": no_operation,
    b"ms": get_motor_status,
    b"mm": move_to_frame,
    b"mp": no_operation,
    b"sm": stop_motor,
    b"sa": stop_all,
    b"jm": unsupported_operation,
    b"im": unsupported_operation,
    b"pr": unsupported_operation,
    b"zm": unsupported_operation,
    b"np": unsupported_operation,
    b"go": unsupported_operation,
}


async def run(in_queue: Queue, reader: StreamReader):
    while True:
        request: bytes = await reader.readuntil(separator=b"\r\n")

        request_type: bytes = request[0:2]
        request_args: [bytes] = request[3 : len(request) - 2].split(b" ")
        logger.debug("Received message: %s", request)

        handler = request_handlers.get(request_type, False)
        if not handler:
            logger.error(f"Received unknown message: {request_type}. Ignoring.")
            continue

        await handler(request_type, request_args, in_queue)
