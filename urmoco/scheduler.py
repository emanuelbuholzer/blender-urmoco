import logging
import multiprocessing as mp
import multiprocessing.context

from urmoco.backend.proc import run as run_backend
from urmoco.dfmoco.proc import run as run_df

logger = logging.getLogger(__name__)


class Scheduler:
    def __init__(self):
        # Setup multiprocessing. Note that this will not work on windows, as fork is not available there ;)
        self.ctx: multiprocessing.context.ForkContext = mp.get_context("fork")

        self.df_in_q = self.ctx.Queue()
        self.df_out_q = self.ctx.Queue()
        self.ur_in_q = self.ctx.Queue()
        self.ur_out_q = self.ctx.Queue()

        self.df_server_proc: mp.Process = None
        self.backend_proc: mp.Process = None

    def start_dfmoco_server(self, config):
        logger.info("Starting the DFMoco server process")
        if self.df_server_proc is None:
            self.df_server_proc = self.ctx.Process(
                target=run_df,
                name="DFMoco server",
                args=(self.df_in_q, self.df_out_q),
                daemon=True,
            )
            self.df_server_proc.start()
            logger.info(
                f"DFMoco server process started with PID {self.df_server_proc.pid}"
            )
        else:
            logger.info(
                f"DFMoco server process already running with PID {self.df_server_proc.pid}"
            )

    def terminate_dfmoco_server(self):
        logger.info("Terminating DFMoco server process")
        if self.df_server_proc is not None:
            self.df_server_proc.terminate()
            logger.info(f"Terminated DFMoco server process")
            self.df_server_proc = None
        else:
            logger.error(f"DFMoco server process already terminated")

    def start_backend(self, config):
        logger.info("Starting backend process")
        if self.backend_proc is None:
            self.backend_proc = self.ctx.Process(
                target=run_backend,
                name="Backend",
                args=(config, self.ur_in_q, self.df_in_q, self.ur_out_q, self.df_out_q),
                daemon=True,
            )
            self.backend_proc.start()
            logger.info(f"Backend process started with PID {self.backend_proc.pid}")
        else:
            logger.info(
                f"Backend process already running with PID {self.backend_proc.pid}"
            )

    def terminate_backend(self):
        logger.info("Terminating backend process")
        if self.backend_proc is not None:
            self.backend_proc.terminate()
            logger.info(f"Terminated backend process")
            self.backend_proc = None
        else:
            logger.error(f"Backend process already terminated")
