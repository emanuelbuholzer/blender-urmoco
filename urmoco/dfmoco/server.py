import asyncio
import logging
import queue

from . import consumer, producer

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


class DFMocoServer:
    def __init__(self, config, dfmoco_in_queue, dfmoco_out_queue):
        self.config = config
        self.in_queue = dfmoco_in_queue
        self.out_queue = dfmoco_out_queue

    async def start(self):
        host = self.config.get("dfmoco.host")
        port = self.config.get("dfmoco.port")
        return await asyncio.start_server(self.handle_connection, host, port)

    async def handle_connection(self, reader, writer):
        logger.info("Connected to Dragonframe")

        writer.write(bytes("hi 1 1 1.2.5\r\n", "ascii"))
        await writer.drain()

        # Clear queues
        while True:
            try:
                self.in_queue.get_nowait()
            except queue.Empty:
                break

        while True:
            try:
                self.out_queue.get_nowait()
            except queue.Empty:
                break

        state = {
            "current_frame": -1,
            "is_moving": False,
        }

        consumer_task = asyncio.create_task(
            consumer.run(self.config, self.in_queue, reader)
        )
        producer_task = asyncio.create_task(
            producer.run(self.config, state, self.out_queue, writer)
        )

        e = await asyncio.gather(consumer_task, producer_task, return_exceptions=True)
        logger.info(f"Disconnected from Dragonframe, an error occurred: {e}")
