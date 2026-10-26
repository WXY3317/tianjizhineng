"""Locate the vendor SDK for repository and installed command-line use."""
import importlib
import os
from pathlib import Path
import sys


def load_sdk(root=None):
    if root is None and 'PYTHON_SDK.GentoRobot' in sys.modules:
        return sys.modules['PYTHON_SDK.GentoRobot']
    if root is None:
        configured = os.environ.get('GENTO_SDK_ROOT')
        candidates = ([Path(configured).expanduser()] if configured else
                      [p / 'tianji-robot-SDK' for p in Path(__file__).resolve().parents])
        root = next((p for p in candidates if (p / 'PYTHON_SDK/GentoRobot.py').is_file()), None)
    else:
        root = Path(root).expanduser().resolve()
    if root is None or not (root / 'PYTHON_SDK/GentoRobot.py').is_file():
        raise ImportError('Set GENTO_SDK_ROOT to the vendor SDK directory containing PYTHON_SDK/GentoRobot.py')
    sys.path.insert(0, str(root))
    return importlib.import_module('PYTHON_SDK.GentoRobot')
