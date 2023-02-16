import asyncio
import logging
import queue
from asyncio import StreamReader, StreamWriter
from multiprocessing import Queue

from . import consumer, producer
from .state import DFMocoState

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


def clear_queue(q: Queue):
    while True:
        try:
            q.get_nowait()
        except queue.Empty:
            break


def get_connection_handler(in_queue: Queue, out_queue: Queue):
    async def handle_connection(reader: StreamReader, writer: StreamWriter):
        logger.info("Connected to Dragonframe")

        clear_queue(in_queue)
        clear_queue(out_queue)

        state: DFMocoState = DFMocoState(current_frame=-1, is_moving=False)

        writer.write(bytes("hi 1 1 1.2.5\r\n", "ascii"))
        writer.write(bytes("mp 1 -1\r\n", "ascii"))
        await writer.drain()

        consumer_task = asyncio.create_task(consumer.run(in_queue, reader))
        producer_task = asyncio.create_task(producer.run(state, out_queue, writer))

        e = await asyncio.gather(consumer_task, producer_task, return_exceptions=True)
        logger.error(f"Disconnected from Dragonframe, an error occurred: {e}")

    return handle_connection


async def create_dfmoco_server(in_queue: Queue, out_queue: Queue):
    connection_handler = get_connection_handler(in_queue, out_queue)
    return await asyncio.start_server(connection_handler, "localhost", 25555)
