import json
import logging
from pathlib import Path

from xdg import XDG_CONFIG_HOME

logger = logging.getLogger(__name__)

defaults = {
    'dfmoco': {
        'host': 'localhost',
        'port': 25555,
        'producer_interval_seconds': 1,
    },
    'robot': {
        'host': '192.168.5.42',
        'payload': 1,
        'freedrive_timeout_seconds': 86400,
        'move_timeout_seconds': 120,
        'target_distance_threshold': 0.001,
        'connect_interval_seconds': 5,
        'sync_interval_seconds': 0.01
    },
}


class Config:

    def __init__(self, config):
        self.config = config

    @staticmethod
    def _get(key, config):
        val = config
        for k in key.split("."):
            if k.isnumeric():
                val = val[int(k)]
            else:
                val = val.get(k)
        if val is None:
            raise AttributeError(f"Key {key} not found in configuration")
        return val

    def load_from_file(self):
        # Resolve config path using XDG and alternatively $HOME/.config followed by urmoco/config.json
        base_path = Path(XDG_CONFIG_HOME)
        if not base_path:
            base_path = Path.home().joinpath('.config')
        config_path = base_path.joinpath('urmoco', 'config.json')

        # Parse the configuration and run the schedule
        try:
            with open(config_path, 'r') as raw_config:
                self.config = json.load(raw_config)
        except FileNotFoundError:
            logging.info("No configuration file found, ignoring.")

    def get(self, key):
        try:
            return self._get(key, self.config)
        except AttributeError as err:
            try:
                val = self._get(key, defaults)
                return val
            except AttributeError:
                raise err
