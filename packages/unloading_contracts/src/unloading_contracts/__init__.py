"""Public shared contract surface."""

from .models import *  # noqa: F401,F403
from .wire import dumps, from_wire, loads, to_wire

__version__ = SCHEMA_VERSION
