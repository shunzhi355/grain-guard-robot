"""PySide6-to-PySide2 compatibility shim for Ubuntu 20.04 / Orange Pi 5 Max.

The board ships PySide2 5.14.0.  All ``grain_sampling_ui`` modules
import ``PySide6`` — this registry shim makes ``import PySide6`` resolve
to the installed PySide2 without editing any source file.
"""
import sys as _sys

import PySide2 as _real

_sys.modules["PySide6"] = _real
for _name in ("QtCore", "QtGui", "QtWidgets", "QtNetwork"):
    try:
        _sys.modules["PySide6." + _name] = getattr(_real, _name)
    except AttributeError:
        pass
