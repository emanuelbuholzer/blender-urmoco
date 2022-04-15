import logging

logger = logging.getLogger(__name__)

defaults = {
    'dashboard': {
        'host': '192.168.5.42',
        'port': 29999,
    }
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

    def get(self, key):
        try:
            return self._get(key, self.config)
        except AttributeError as err:
            try:
                val = self._get(key, defaults)
                return val
            except AttributeError:
                raise err
