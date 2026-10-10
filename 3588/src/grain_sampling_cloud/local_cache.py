"""
Local persistent cache for unsent HTTP API requests.

Uses SQLite to store POST requests that could not be delivered,
allowing them to be flushed later when connectivity is restored.
Thread-safe.
"""

import json
import logging
import os
import sqlite3
import threading
from contextlib import contextmanager
from typing import Any, Dict, Iterator, List, Optional

logger = logging.getLogger(__name__)


class LocalCache:
    """Thread-safe SQLite-backed cache for queuing failed HTTP requests.

    Requests are stored as ``(path, data_json)`` rows.  Once they are
    successfully sent via the provided ``CloudHttpClient`` they are
    removed from the cache.

    Args:
        db_path: Path to the SQLite database file. If ``None`` a default
            path ``~/.grain_sampling/http_cache.db`` is used.
    """

    DEFAULT_DB_PATH = os.path.join(
        os.path.expanduser("~"), ".grain_sampling", "http_cache.db"
    )

    def __init__(self, db_path: Optional[str] = None) -> None:
        self._db_path = db_path or self.DEFAULT_DB_PATH
        self._lock = threading.Lock()

        # Ensure the parent directory exists
        db_dir = os.path.dirname(self._db_path)
        if db_dir:
            os.makedirs(db_dir, exist_ok=True)

        self._init_db()

    # ------------------------------------------------------------------
    # Database setup
    # ------------------------------------------------------------------

    def _init_db(self) -> None:
        """Create the database and table if they don't exist."""
        with self._get_connection() as conn:
            conn.execute(
                """
                CREATE TABLE IF NOT EXISTS outbox (
                    id          INTEGER PRIMARY KEY AUTOINCREMENT,
                    path        TEXT    NOT NULL,
                    data        TEXT    NOT NULL,
                    created_at  REAL    NOT NULL DEFAULT (strftime('%s','now'))
                )
                """
            )
            conn.commit()

    @contextmanager
    def _get_connection(self) -> Iterator[sqlite3.Connection]:
        """Yield a thread-safe SQLite connection (managed by the lock)."""
        conn = sqlite3.connect(self._db_path, check_same_thread=False)
        conn.execute("PRAGMA journal_mode=WAL")
        conn.execute("PRAGMA synchronous=NORMAL")
        try:
            yield conn
        finally:
            conn.close()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def store(
        self,
        path: str,
        data: Dict[str, Any],
    ) -> None:
        """Enqueue a failed HTTP request into the local cache.

        The request will be stored persistently and can be flushed later.

        Args:
            path: API path (e.g. ``/api/tasks``).
            data: Dictionary payload (will be JSON-serialized).
        """
        data_json = json.dumps(data, ensure_ascii=False)
        with self._lock:
            with self._get_connection() as conn:
                conn.execute(
                    "INSERT INTO outbox (path, data) VALUES (?, ?)",
                    (path, data_json),
                )
                conn.commit()
        logger.debug("Cached request for path %s (queue now has %d items)", path, self.get_pending_count())

    def flush_all(self, http_client: Any) -> int:
        """Send all cached requests via the provided HTTP client.

        Successfully sent requests are removed from the cache.  If
        sending fails for a request, it remains in the cache for a
        later retry.

        Args:
            http_client: An object with a ``post(path, data)`` method
                (e.g. ``CloudHttpClient`` from ``http_client.py``).

        Returns:
            The number of requests that were successfully flushed.
        """
        flushed = 0
        with self._lock:
            with self._get_connection() as conn:
                rows = conn.execute(
                    "SELECT id, path, data FROM outbox ORDER BY id ASC"
                ).fetchall()

                for row in rows:
                    req_id, path, data_json = row
                    try:
                        data_dict = json.loads(data_json)
                        http_client.post(path, data_dict)
                        conn.execute("DELETE FROM outbox WHERE id = ?", (req_id,))
                        flushed += 1
                        logger.debug("Flushed cached request id=%d to %s", req_id, path)
                    except Exception as exc:
                        logger.warning(
                            "Failed to flush request id=%d to %s: %s",
                            req_id, path, exc,
                        )
                conn.commit()

        total = flushed + self.get_pending_count()
        logger.info("Flushed %d / %d cached requests", flushed, total)
        return flushed

    def get_pending_count(self) -> int:
        """Return the number of messages currently in the cache.

        Returns:
            Count of pending (unsent) messages.
        """
        with self._lock:
            with self._get_connection() as conn:
                row = conn.execute("SELECT COUNT(*) FROM outbox").fetchone()
                return row[0] if row else 0

    def clear(self) -> None:
        """Remove all messages from the cache."""
        with self._lock:
            with self._get_connection() as conn:
                conn.execute("DELETE FROM outbox")
                conn.commit()
        logger.info("Cleared all cached messages")

    def get_all(self) -> List[Dict[str, Any]]:
        """Return all cached requests (for inspection / debugging).

        Returns:
            List of dicts with keys ``id``, ``path``, ``data``,
            ``created_at``.
        """
        with self._lock:
            with self._get_connection() as conn:
                rows = conn.execute(
                    "SELECT id, path, data, created_at FROM outbox ORDER BY id ASC"
                ).fetchall()
                return [
                    {
                        "id": r[0],
                        "path": r[1],
                        "data": json.loads(r[2]),
                        "created_at": r[3],
                    }
                    for r in rows
                ]
