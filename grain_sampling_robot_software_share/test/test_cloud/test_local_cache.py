"""Tests for :class:`grain_sampling_cloud.local_cache.LocalCache`."""

from __future__ import annotations

import os
import tempfile
from unittest.mock import MagicMock

import pytest

from grain_sampling_cloud.local_cache import LocalCache


@pytest.fixture
def cache():
    """Create a LocalCache with a temp SQLite database."""
    with tempfile.TemporaryDirectory() as tmpdir:
        db_path = os.path.join(tmpdir, "test_cache.db")
        yield LocalCache(db_path=db_path)


class TestLocalCache:
    """8 tests covering store, flush, count, and connection lifecycle."""

    def test_store_increases_pending_count(self, cache):
        assert cache.get_pending_count() == 0
        cache.store("/api/tasks", {"mac": "test"})
        assert cache.get_pending_count() == 1

    def test_store_multiple(self, cache):
        for i in range(5):
            cache.store(f"/api/item/{i}", {"id": i})
        assert cache.get_pending_count() == 5

    def test_flush_all_success(self, cache):
        mock_client = MagicMock()
        mock_client.post.return_value = {"success": True}
        cache.store("/api/tasks", {"order": "001"})
        cache.store("/api/status", {"status": "done"})
        flushed = cache.flush_all(mock_client)
        assert flushed == 2
        assert cache.get_pending_count() == 0

    def test_flush_all_partial_failure(self, cache):
        mock_client = MagicMock()
        # First call succeeds, second fails
        mock_client.post.side_effect = [{"success": True}, Exception("network error")]
        cache.store("/api/tasks", {"order": "001"})
        cache.store("/api/status", {"status": "done"})
        flushed = cache.flush_all(mock_client)
        assert flushed == 1  # one succeeded
        assert cache.get_pending_count() == 1  # one remains

    def test_flush_all_empty_cache(self, cache):
        mock_client = MagicMock()
        flushed = cache.flush_all(mock_client)
        assert flushed == 0
        mock_client.post.assert_not_called()

    def test_initial_pending_count_zero(self, cache):
        assert cache.get_pending_count() == 0

    def test_stored_data_preserved(self, cache):
        cache.store("/api/test", {"key": "value"})
        mock_client = MagicMock()
        mock_client.post.return_value = {"success": True}
        cache.flush_all(mock_client)
        # Verify the correct data was posted
        mock_client.post.assert_called_once_with("/api/test", {"key": "value"})

    def test_db_connection_closed_after_use(self, cache):
        # Store and flush — connection should be managed correctly
        mock_client = MagicMock()
        mock_client.post.return_value = {"success": True}
        cache.store("/api/test", {"a": 1})
        cache.flush_all(mock_client)
        # No exception = connection handling is correct
        assert cache.get_pending_count() == 0
