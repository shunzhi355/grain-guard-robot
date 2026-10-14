"""Persistent current-task state for reboot recovery.

Stores the currently accepted / in-progress task locally so that after a
device reboot the UI can detect "a task was still running when the device
shut down" and offer the operator to abandon it on the cloud
(``POST /report status=ABANDON``).

File: ``~/.grain_robot/current_task.json`` (same dir as settings.json).
"""

from __future__ import annotations

import json
import logging
import os
import threading
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)

CURRENT_TASK_JSON_PATH = os.path.expanduser("~/.grain_robot/current_task.json")

_lock = threading.Lock()


def _write(data: Optional[Dict[str, Any]]) -> None:
    """Atomically write the current-task JSON file (or remove it)."""
    try:
        path = CURRENT_TASK_JSON_PATH
        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        tmp = path + ".tmp"
        with _lock:
            if data is None:
                if os.path.exists(tmp):
                    os.remove(tmp)
                if os.path.exists(path):
                    os.remove(path)
                return
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            os.replace(tmp, path)
    except Exception:
        logger.exception("Failed to persist current task state")


def save_current_task(task_id: str, source: str = "cloud", **extra: Any) -> None:
    """Persist the currently in-progress task.

    Args:
        task_id: Cloud order id (or local task id) of the running task.
        source: ``"cloud"`` or ``"local"``.
        **extra: Optional context (e.g. aojian, warehouse) kept for display.
    """
    payload: Dict[str, Any] = {"task_id": task_id, "source": source, **extra}
    _write(payload)
    logger.info("Current task persisted: %s (%s)", task_id, source)


def load_current_task() -> Optional[Dict[str, Any]]:
    """Return the persisted current-task dict, or ``None`` if none saved."""
    try:
        path = CURRENT_TASK_JSON_PATH
        if not os.path.exists(path):
            return None
        with _lock:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        if isinstance(data, dict):
            return data
        return None
    except Exception:
        logger.exception("Failed to load current task state")
        return None


def clear_current_task() -> None:
    """Remove the persisted current-task record."""
    _write(None)
    logger.info("Current task state cleared")
