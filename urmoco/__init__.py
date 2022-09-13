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
    __import__('pip._internal')._internal.main(['install', 'ur_rtde', 'xdg'])

    from .blender.operators import get_operators
    from .blender.state import URMocoState
    from .blender.sync import get_urmoco_sync
    from .blender.panel import URMocoPanel
    from .blender.messagebox import MessageBox
    from .blender.preferences import Preferences, get_preferences_property_group
    from .dfmoco.proc import run as run_dfmoco
    from .backend.proc import run as run_urmoco
    from urmoco.scheduler import Scheduler
    from .config import Config
    import logging
    import bpy
    from logging.handlers import TimedRotatingFileHandler
    from pathlib import Path
    from xdg import XDG_CACHE_HOME

    # Setup logging over stdout and a rotating log file
    logger = logging.getLogger(__name__)
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(name)s - %(message)s')

    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(formatter)
    logger.addHandler(stream_handler)

    def file_handler(filename):
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(name)s - %(message)s')
        path = Path(XDG_CACHE_HOME).joinpath(filename)
        os.system(f"mkdir -p {XDG_CACHE_HOME}")
        os.system(f"touch {path}")
        handler = TimedRotatingFileHandler(path, when="d", interval=1, backupCount=7)
        handler.setFormatter(formatter)
        return handler
    logger.addHandler(file_handler("urmoco.log"))

    # Initialise and load configuration.
    config = Config({})
    config.load_from_file()

    scheduler = Scheduler()

    # Load bpy types which are dependent on queues or configs
    operators = get_operators(config, scheduler)
    sync = get_urmoco_sync(config, scheduler.ur_out_q)
    URMocoPreferences = get_preferences_property_group(config)

    def register():
        # Register bpy classes
        bpy.utils.register_class(URMocoState)
        bpy.types.WindowManager.urmoco_state = bpy.props.PointerProperty(type=URMocoState)

        bpy.utils.register_class(URMocoPreferences)
        bpy.types.WindowManager.urmoco_preferences = bpy.props.PointerProperty(type=URMocoPreferences)

        bpy.utils.register_class(Preferences)
        bpy.utils.register_class(MessageBox)

        for operator in operators:
            bpy.utils.register_class(operator)

        bpy.utils.register_class(URMocoPanel)

        bpy.app.timers.register(sync, persistent=True)

    def unregister():
        # Unregister bpy classes
        bpy.utils.unregister_class(URMocoPanel)

        for operator in operators:
            bpy.utils.unregister_class(operator)

        bpy.utils.unregister_class(Preferences)
        bpy.utils.unregister_class(MessageBox)

        bpy.utils.unregister_class(URMocoPreferences)
        bpy.utils.unregister_class(URMocoState)

        # Terminate processes
        scheduler.terminate_dfmoco_server()
        scheduler.terminate_backend()
