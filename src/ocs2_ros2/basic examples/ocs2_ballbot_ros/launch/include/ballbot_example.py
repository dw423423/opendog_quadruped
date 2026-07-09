"""Import shim for the shared ballbot launch fragments."""

import importlib.util
from pathlib import Path


_LAUNCH_FILE = Path(__file__).with_name("ballbot_example.launch.py")
_SPEC = importlib.util.spec_from_file_location("_ballbot_example_launch", _LAUNCH_FILE)
if _SPEC is None or _SPEC.loader is None:
    raise ImportError(f"Cannot load launch helper: {_LAUNCH_FILE}")

_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)

declare_path_arguments = _MODULE.declare_path_arguments
node_parameters = _MODULE.node_parameters
visualize_launch = _MODULE.visualize_launch
