import os
import sys

bl_info = {
    "name": "URMoco",
    "description": "Blender Universal Robot Motion Control",
    "author": "Emanuel Buholzer",
    "version": (0, 1, 0),
    "blender": (3, 1, 2),
    "location": "View3D > Tool Shelf > URMoco",
    "doc_url": "https://https://github.com/emanuelbuholzer/blender-ur-moco",
    "tracker_url": "https://github.com/emanuelbuholzer/blender-ur-moco/issues",
    "support": "COMMUNITY",
    "category": "Motion Control"
}

# We want to execute this code only if we are running in blender
if "bpy" in sys.modules:

    # Install dependencies. This takes a while..
    __import__('ensurepip')._bootstrap()
    __import__('pip._internal')._internal.main(['install', '-U', 'pip', 'setuptools', 'wheel'])
    __import__('pip._internal')._internal.main(['install', 'ur_rtde', 'roboticstoolbox-python', 'xdg'])

    from .blender.operators import get_operators
    from .blender.state import URMocoState
    from .blender.sync import get_urmoco_sync
    from .blender.panel import URMocoPanel
    from .blender.messagebox import MessageBox
    from .dfmoco.proc import run as run_dfmoco
    from .backend.proc import run as run_urmoco
    from .config import Config
    import multiprocessing as mp
    import logging
    import bpy
    from logging.handlers import TimedRotatingFileHandler
    from pathlib import Path
    from xdg import XDG_CACHE_HOME
    from bpy.app.handlers import persistent

    # Setup logging over stdout and a rotating log file
    logger = logging.getLogger(__name__)
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(name)s - %(message)s')

    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(formatter)
    logger.addHandler(stream_handler)

    def file_handler(filename):
        path = Path(XDG_CACHE_HOME).joinpath(filename)
        os.system(f"mkdir -p {XDG_CACHE_HOME}")
        os.system(f"touch {path}")
        handler = TimedRotatingFileHandler(path, when="d", interval=1, backupCount=7)
        handler.setFormatter(formatter)
        return handler
    logger.addHandler(file_handler("dfmoco2ur.log"))

    # Initialise and load configuration.
    config = Config({})
    config.load_from_file()

    # Setup multiprocessing. Note that this will not work on windows, as fork is not available there,
    # which is needed to schedule process from within Blender.
    mp_context = mp.get_context('fork')

    dfmoco_in_queue = mp_context.Queue()
    dfmoco_out_queue = mp_context.Queue()
    urmoco_in_queue = mp_context.Queue()
    urmoco_out_queue = mp_context.Queue()

    procs = {
        'dfmoco': None,
        'urmoco': None
    }

    # Load bpy types which are dependent on queues or configs
    operators = get_operators(config, urmoco_in_queue, urmoco_out_queue)
    sync = get_urmoco_sync(config, urmoco_in_queue, urmoco_out_queue)

    @persistent
    def load_handler(dummy):
        urmoco_in_queue.put({"type": "hi"})

    bpy.app.handlers.load_post.append(load_handler)

    def register():
        # Start processes
        if procs["dfmoco"] is None:
            procs["dfmoco"] = mp_context.Process(target=run_dfmoco, name="DFMoco Server",
                                                args=(config, dfmoco_in_queue, dfmoco_out_queue), daemon=True)
        procs["dfmoco"].start()
        logger.info(f"DFMoco process started with PID {procs['dfmoco'].pid}")

        if procs["urmoco"] is None:
            procs["urmoco"] = mp_context.Process(target=run_urmoco, name="URMoco Backend",
                                                 args=(config, urmoco_in_queue, dfmoco_in_queue, urmoco_out_queue, dfmoco_out_queue),
                                                 daemon=True)
        procs["urmoco"].start()
        logger.info(f"URMoco process started with PID {procs['urmoco'].pid}")

        # Register bpy classes
        bpy.utils.register_class(URMocoState)
        bpy.types.WindowManager.urmoco_state = bpy.props.PointerProperty(type=URMocoState)

        bpy.utils.register_class(MessageBox)

        for operator in operators:
            bpy.utils.register_class(operator)

        bpy.utils.register_class(URMocoPanel)

        bpy.app.timers.register(sync, persistent=True)

        # Say hi to the backend to get the initial state
        urmoco_in_queue.put({"type": "hi"})

    def unregister():
        # Unregister bpy classes
        bpy.utils.unregister_class(URMocoPanel)

        for operator in operators:
            bpy.utils.unregister_class(operator)

        bpy.utils.unregister_class(MessageBox)

        bpy.utils.unregister_class(URMocoState)

        bpy.app.timers.unregister(sync)

        # Terminate processes
        procs["dfmoco"].terminate()
        logger.info("Terminated DFMoco process")
        procs["dfmoco"] = None

        procs["urmoco"].terminate()
        logger.info("Terminated URMoco process")
        procs["urmoco"] = None
