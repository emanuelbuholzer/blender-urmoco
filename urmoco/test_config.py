import pytest
from .config import Config


def test_given_value():
    config = Config({"dashboard": {"host": "172.168.5.42"}})
    ur_host = config.get("dashboard.host")
    assert ur_host == "172.168.5.42"


def test_hard_coded_value():
    config = Config({})
    dfmoco_port = config.get("dfmoco.port")
    assert dfmoco_port == 25555


def test_array_without_string_name():
    va = {"va": 1}
    bla = {"bla": 2}
    config = Config({"hal": [va, bla]})
    va = config.get("hal.0.va")
    assert va == 1


def test_parent_access():
    config = Config({"hal": {"va": 1, "bla": 2}})
    hal = config.get("hal")
    assert hal == {"va": 1, "bla": 2}


def test_key_not_found():
    with pytest.raises(AttributeError):
        config = Config({})
        config.get("halli.galli")
