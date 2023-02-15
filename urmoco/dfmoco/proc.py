import asyncio
import logging

from .server import create_dfmoco_server

logger = logging.getLogger(__name__)


async def run_dfmoco_server(dfmoco_in_queue, dfmoco_out_queue):
    dfmoco_server = await create_dfmoco_server(dfmoco_in_queue, dfmoco_out_queue)

    addr = dfmoco_server.sockets[0].getsockname()
    logger.info(f"DFMoco server started on {addr[0]}:{addr[1]}")

    await dfmoco_server.serve_forever()


def run(dfmoco_in_queue, dfmoco_out_queue):
    asyncio.run(run_dfmoco_server(dfmoco_in_queue, dfmoco_out_queue))
