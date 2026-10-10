"""Page manager using QStackedWidget for page navigation.

Provides push/pop navigation with a maintained stack for back support.
"""

from __future__ import annotations

from typing import Dict, List, Optional

from PySide2.QtCore import QObject, Signal, Slot
from PySide2.QtWidgets import QStackedWidget, QWidget


class PageManager(QObject):
    """Manages page navigation with a QStackedWidget.

    Supports register, push, pop, go_home patterns with page stack tracking
    and a ``page_changed`` signal for external listeners.
    """

    page_changed = Signal(str)

    def __init__(self, stacked_widget: QStackedWidget, parent: Optional[QObject] = None) -> None:
        super().__init__(parent)
        self._stack: QStackedWidget = stacked_widget
        self._pages: Dict[str, QWidget] = {}
        self._history: List[str] = []
        self._home_name: str = ""

    # ── Registration ───────────────────────────────────────

    def register_page(self, name: str, widget: QWidget) -> int:
        """Register a page widget with a unique name.

        Args:
            name: Unique page identifier (e.g. ``"main"``).
            widget: The QWidget instance for this page.

        Returns:
            Index of the added widget in the stack.
        """
        self._pages[name] = widget
        return self._stack.addWidget(widget)

    # ── Navigation ─────────────────────────────────────────

    def push(self, name: str) -> None:
        """Push a page onto the navigation stack and display it."""
        if name not in self._pages:
            raise KeyError(f"Page not registered: {name}")

        if self.current_page_name() == name:
            return

        self._history.append(name)
        self._stack.setCurrentWidget(self._pages[name])
        self.page_changed.emit(name)

    def pop(self) -> Optional[str]:
        """Pop the current page and go back to the previous one.

        Returns:
            Name of the page navigated to, or ``None`` if at root.
        """
        if len(self._history) <= 1:
            # At root — cannot go back further
            return None

        # Remove current
        self._history.pop()
        # Get previous
        prev = self._history[-1]

        self._stack.setCurrentWidget(self._pages[prev])
        self.page_changed.emit(prev)
        return prev

    def go_home(self) -> None:
        """Navigate to the home page, clearing the navigation history."""
        if not self._home_name or self._home_name not in self._pages:
            return
        self._history = [self._home_name]
        self._stack.setCurrentWidget(self._pages[self._home_name])
        self.page_changed.emit(self._home_name)

    def set_home(self, name: str) -> None:
        """Designate a registered page as the home page.

        Does NOT navigate — call ``go_home()`` after.
        """
        if name not in self._pages:
            raise KeyError(f"Page not registered: {name}")
        self._home_name = name

    # ── Queries ────────────────────────────────────────────

    def current_page_name(self) -> Optional[str]:
        """Return the name of the currently displayed page."""
        return self._history[-1] if self._history else None

    def page_names(self) -> List[str]:
        """Return list of all registered page names."""
        return list(self._pages.keys())

    # ── Helpers ────────────────────────────────────────────

    def _get_current_name(self) -> Optional[str]:
        current_widget = self._stack.currentWidget()
        if current_widget is None:
            return None
        for name, w in self._pages.items():
            if w is current_widget:
                return name
        return None
