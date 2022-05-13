import logging
import asyncio
from .server import DFMocoServer

logger = logging.getLogger(__name__)


async def run_dfmoco_server(config, dfmoco_in_queue, dfmoco_out_queue):
    dfmoco_server = await DFMocoServer(config, dfmoco_in_queue, dfmoco_out_queue).start()

    addr = dfmoco_server.sockets[0].getsockname()
    logger.info(f"DFMoco server started on {addr[0]}:{addr[1]}")

    await dfmoco_server.serve_forever()


def run(config, dfmoco_in_queue, dfmoco_out_queue):
    asyncio.run(run_dfmoco_server(config, dfmoco_in_queue, dfmoco_out_queue))
