import logging
import multiprocessing as mp
import dfmoco.proc as dfmoco
import urmoco.proc as urmoco
import api.proc as api

logger = logging.getLogger(__name__)


def run(config):
    # Use spawn instead of fork to get the same behaviour across diffrent operating systems
    mp_context = mp.get_context('spawn')

    dfmoco_in_queue = mp_context.Queue()
    dfmoco_out_queue = mp_context.Queue()

    urmoco_in_queue = mp_context.Queue()
    urmoco_out_queue = mp_context.Queue()

    dfmoco_proc = mp_context.Process(target=dfmoco.run, name="DFMoco server",
                                     args=(config, dfmoco_in_queue, dfmoco_out_queue), daemon=True)
    dfmoco_proc.start()
    logger.info("DFMoco server started with PID {}".format(dfmoco_proc.pid))

    urmoco_proc = mp_context.Process(target=api.run, name="API server",
                                     args=(config, urmoco_in_queue, urmoco_out_queue), daemon=True)
    urmoco_proc.start()
    logger.info("API server started with PID {}".format(urmoco_proc.pid))

    urmoco_proc = mp_context.Process(target=urmoco.run, name="URMoco", args=(
    config, urmoco_in_queue, dfmoco_in_queue, urmoco_out_queue, dfmoco_out_queue), daemon=True)
    urmoco_proc.start()
    logger.info("URMoco started with PID {}".format(urmoco_proc.pid))

    mp.connection.wait([dfmoco_proc.sentinel, urmoco_proc.sentinel, urmoco_proc.sentinel])
