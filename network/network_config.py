"""
network_config.py
Loads network/config.yaml and exposes the Pi's and each Arduino's
IP/port settings. Edit config.yaml when an address changes — nothing
here should need editing.

Usage:
    from network.network_config import pi, arduino
    pi()["ip"]
    arduino("rf")["data_port"]
"""

from pathlib import Path

import yaml

CONFIG_PATH = Path(__file__).resolve().parent / "config.yaml"


def load_config(path: Path = CONFIG_PATH) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


_config = load_config()


def pi() -> dict:
    return _config["pi"]


def arduino(name: str) -> dict:
    try:
        return _config["arduinos"][name]
    except KeyError:
        raise KeyError(f"No arduino named '{name}' in {CONFIG_PATH}") from None


def fcp() -> dict:
    return _config["fcp"]
