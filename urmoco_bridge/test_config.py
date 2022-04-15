import pytest
from .config import Config


def test_given_value():
    config = Config({"dashboard": {"host": "172.168.5.42"}})
    ur_host = config.get("dashboard.host")
    assert ur_host == "172.168.5.42"


def test_hard_coded_value():
    config = Config({})
    ur_host = config.get("dashboard.port")
    assert ur_host == 29999


def test_array_without_string_name():
    va = {"va": 1}
    bla = {"bla": 2}
    config = Config({"hal": [va, bla]})
    va = config.get("hal.0.va")
    assert va == 1


def test_parent_access():
    config = Config({"hal": {"va": 1, "bla": 2}})
    hal = config.get("hal")
    print(hal)
    assert hal == {"va": 1, "bla": 2}


def test_key_not_found():
    with pytest.raises(AttributeError):
        config = Config({})
        config.get("halli.galli")
