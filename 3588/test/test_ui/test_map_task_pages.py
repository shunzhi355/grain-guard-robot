"""Unit tests for MapPage (廒间地图) and TaskListPage (任务列表)."""

from __future__ import annotations

import pytest
from PySide2.QtCore import Qt
from PySide2.QtWidgets import QApplication

from grain_sampling_ui.pages.map_page import MapPage, _UploadDialog
import grain_sampling_ui.pages.task_list_page as task_list_module
from grain_sampling_ui.pages.task_list_page import (
    SKIP_MAPPING_WAREHOUSE,
    TaskListPage,
    _CreateTaskDialog,
    _TaskItemWidget,
    SAMPLE_TASKS,
)
from grain_sampling_cloud.protocol import OrderInfo


# ── Qt Application fixture (module-scoped, shared) ──────────────────────


@pytest.fixture(scope="module")
def qapp() -> QApplication:
    """Create one QApplication for the entire test module."""
    app = QApplication.instance()
    if app is None:
        app = QApplication([])
    yield app
    # Do NOT call app.quit() — other tests may share it


# ═══════════════════════════════════════════════════════════════════════════
# MapPage Tests
# ═══════════════════════════════════════════════════════════════════════════


class TestMapPage:
    """Tests for MapPage (廒间地图)."""

    @pytest.fixture
    def page(self, qapp: QApplication) -> MapPage:  # noqa: ARG002
        return MapPage()

    def test_page_creation(self, page: MapPage) -> None:
        """Page should be created with correct object name and layout."""
        assert page.objectName() == "map_page"
        assert page.layout() is not None

    def test_page_title(self, page: MapPage) -> None:
        """Page title should display '廒间地图'."""
        from PySide2.QtWidgets import QLabel

        title_found = False
        for child in page.findChildren(QLabel):
            if child.objectName() == "page_name":
                assert child.text() == "廒间地图"
                title_found = True
                break
        assert title_found, "page_name QLabel not found"

    def test_upload_dialog_creation(self, qapp: QApplication) -> None:  # noqa: ARG002
        """Upload dialog should be created with correct widgets."""
        dlg = _UploadDialog()
        assert dlg.windowTitle() == "上传点云地图"
        assert dlg.get_height() == 0.5

        # Preview should not be visible before clicking preview
        # (We test this via signal emission below)

    def test_upload_dialog_preview_flow(self, qapp: QApplication) -> None:  # noqa: ARG002
        """Preview button should emit request_preview signal and show upload button."""
        dlg = _UploadDialog()
        preview_signals: list[float] = []
        dlg.request_preview.connect(lambda h: preview_signals.append(h))

        dlg._on_preview()
        assert len(preview_signals) == 1
        assert preview_signals[0] == 0.5

        # After preview, upload button should be visible
        assert dlg._upload_btn is not None
        assert dlg._upload_btn.isVisible()

    def test_upload_dialog_height_range(self, qapp: QApplication) -> None:  # noqa: ARG002
        """Height spinbox should accept values in [0.1, 10.0]."""
        dlg = _UploadDialog()
        spin = dlg._height_spin
        assert spin.minimum() == 0.1
        assert spin.maximum() == 10.0
        assert spin.value() == 0.5

    def test_upload_button_emits_signal(self, qapp: QApplication) -> None:  # noqa: ARG002
        """Upload button should emit upload_map signal."""
        dlg = _UploadDialog()
        # First show upload button via preview
        dlg._on_preview()

        upload_signals: list[float] = []
        dlg.upload_map.connect(lambda h: upload_signals.append(h))

        dlg._on_upload()
        assert len(upload_signals) == 1
        assert upload_signals[0] == 0.5

    def test_upload_progress(self, qapp: QApplication) -> None:  # noqa: ARG002
        """set_upload_progress should update progress bar."""
        dlg = _UploadDialog()
        dlg.set_upload_progress(50)
        assert dlg._progress_bar is not None
        assert dlg._progress_bar.value() == 50

    def test_upload_success(self, qapp: QApplication) -> None:  # noqa: ARG002
        """show_upload_success should display success message."""
        dlg = _UploadDialog()
        dlg.show_upload_success()
        assert dlg._progress_bar is not None
        assert dlg._progress_bar.value() == 100
        assert dlg._status_label is not None
        assert dlg._status_label.isVisible()
        assert "成功" in dlg._status_label.text()

    def test_upload_failure(self, qapp: QApplication) -> None:  # noqa: ARG002
        """show_upload_failure should display error message."""
        dlg = _UploadDialog()
        dlg.show_upload_failure("网络超时")
        assert dlg._status_label is not None
        assert dlg._status_label.isVisible()
        assert "失败" in dlg._status_label.text()
        assert "网络超时" in dlg._status_label.text()

    def test_page_upload_bridge(self, page: MapPage) -> None:
        """MapPage should forward signals from upload dialog."""
        preview_signals: list[float] = []
        upload_signals: list[float] = []
        page.request_preview.connect(lambda h: preview_signals.append(h))
        page.upload_map.connect(lambda h: upload_signals.append(h))

        # Simulate the dialog flow via public methods
        page._on_upload_started(1.5)
        assert len(upload_signals) == 1
        assert upload_signals[0] == 1.5

    def test_page_upload_progress_bridge(self, page: MapPage) -> None:
        """update_upload_progress should not crash when no dialog is open."""
        # No dialog open — should not raise
        page.update_upload_progress(30)

    def test_page_success_failure_bridge(self, page: MapPage) -> None:
        """show_upload_success/failure should not crash when no dialog is open."""
        page.show_upload_success()  # no dialog — no-op
        page.show_upload_failure("测试错误")  # no dialog — no-op

    def test_get_current_upload_height_no_dialog(self, page: MapPage) -> None:
        """get_current_upload_height should return None when no dialog is open."""
        assert page.get_current_upload_height() is None


# ═══════════════════════════════════════════════════════════════════════════
# TaskListPage Tests
# ═══════════════════════════════════════════════════════════════════════════


class TestTaskListPage:
    """Tests for TaskListPage (任务列表)."""

    @pytest.fixture
    def page(self, qapp: QApplication) -> TaskListPage:  # noqa: ARG002
        return TaskListPage()

    def test_page_creation(self, page: TaskListPage) -> None:
        """Page should be created with correct object name."""
        assert page.objectName() == "task_list_page"
        assert page.layout() is not None

    def test_page_title(self, page: TaskListPage) -> None:
        """Page title should display '任务列表'."""
        from PySide2.QtWidgets import QLabel

        title_found = False
        for child in page.findChildren(QLabel):
            if child.objectName() == "page_name":
                assert child.text() == "任务列表"
                title_found = True
                break
        assert title_found, "page_name QLabel not found"

    def test_filter_buttons_present(self, page: TaskListPage) -> None:
        """All four filter buttons should be present."""
        assert len(page._filter_buttons) == 4
        for label in ["全部", "未完成", "进行中", "已完成"]:
            assert label in page._filter_buttons
            assert page._filter_buttons[label].isCheckable()

    def test_default_filter_is_all(self, page: TaskListPage) -> None:
        """Default active filter should be '全部'."""
        assert page.active_filter == "全部"
        assert page._filter_buttons["全部"].isChecked()

    def test_filter_all_shows_all_tasks(self, page: TaskListPage) -> None:
        """With '全部' filter, all 5 sample tasks should be visible."""
        assert page._list_widget.count() == 5

    def test_filter_unfinished(self, page: TaskListPage) -> None:
        """With '未完成' filter, only unfinished tasks should be shown."""
        page._on_filter_changed("未完成")
        assert page.active_filter == "未完成"
        tasks = page.get_filtered_tasks()
        assert all(t["status"] == "未完成" for t in tasks)
        assert len(tasks) == sum(1 for t in SAMPLE_TASKS if t["status"] == "未完成")

    def test_filter_in_progress(self, page: TaskListPage) -> None:
        """With '进行中' filter, only in-progress tasks should be shown."""
        page._on_filter_changed("进行中")
        assert page.active_filter == "进行中"
        tasks = page.get_filtered_tasks()
        assert all(t["status"] == "进行中" for t in tasks)

    def test_filter_completed(self, page: TaskListPage) -> None:
        """With '已完成' filter, only completed tasks should be shown."""
        page._on_filter_changed("已完成")
        assert page.active_filter == "已完成"
        tasks = page.get_filtered_tasks()
        assert all(t["status"] == "已完成" for t in tasks)

    def test_task_selection_signal(self, page: TaskListPage) -> None:
        """Clicking '选择此任务' should emit task_selected signal with task ID."""
        selected: list[str] = []
        page.task_selected.connect(lambda tid: selected.append(tid))

        # Find a task item widget and simulate selection
        for i in range(page._list_widget.count()):
            item = page._list_widget.item(i)
            widget: _TaskItemWidget = page._list_widget.itemWidget(item)
            widget.task_selected.emit(widget._task_id)
            break  # Only test the first one

        assert len(selected) == 1
        assert selected[0] in [t["id"] for t in SAMPLE_TASKS]

    def test_filter_button_style_updates(self, page: TaskListPage) -> None:
        """Switching filter should update button styles."""
        # Switch to '进行中'
        page._on_filter_changed("进行中")
        assert page._filter_buttons["进行中"].isChecked()
        assert not page._filter_buttons["全部"].isChecked()

        # Switch to '已完成'
        page._on_filter_changed("已完成")
        assert page._filter_buttons["已完成"].isChecked()
        assert not page._filter_buttons["进行中"].isChecked()

    def test_update_task_status(self, page: TaskListPage) -> None:
        """update_task_status should change a task's status and refresh the list."""
        task_id = SAMPLE_TASKS[0]["id"]
        page.update_task_status(task_id, "已完成")
        for t in page._all_tasks:
            if t["id"] == task_id:
                assert t["status"] == "已完成"
                break
        else:
            pytest.fail("Task not found after status update")

    def test_add_task(self, page: TaskListPage) -> None:
        """add_task should append a new task and refresh the list."""
        initial_count = page._list_widget.count()
        page.add_task({
            "id": "T-TEST-001",
            "warehouse": "测试仓",
            "status": "未完成",
            "created": "2025-07-10 12:00",
        })
        assert page._list_widget.count() == initial_count + 1


# ═══════════════════════════════════════════════════════════════════════════
# CreateTaskDialog Tests
# ═══════════════════════════════════════════════════════════════════════════


class TestCreateTaskDialog:
    """Tests for _CreateTaskDialog."""

    @pytest.fixture
    def dialog(
        self,
        qapp: QApplication,  # noqa: ARG002
        monkeypatch: pytest.MonkeyPatch,
        tmp_path,
    ) -> _CreateTaskDialog:
        monkeypatch.setattr(task_list_module, "PCD_DIR", str(tmp_path))
        monkeypatch.delenv("GRAIN_SAMPLING_UI_SKIP_MAPPING", raising=False)
        (tmp_path / "粮仓D区-01号_20250710.pcd").touch()
        return _CreateTaskDialog()

    def test_dialog_creation(self, dialog: _CreateTaskDialog) -> None:
        """Dialog should have correct title and default values."""
        assert dialog.windowTitle() == "本地创建工单"
        assert dialog._depth_inputs[0].currentText() == "2.0 米"
        assert dialog._depth_inputs[1].currentText() == "无"
        assert dialog._depth_inputs[2].currentText() == "无"
        # Waypoint list should have one default item
        assert dialog._waypoint_list.count() == 1

    def test_dialog_rejects_empty_warehouse(self, dialog: _CreateTaskDialog) -> None:
        """Creating with empty warehouse should show warning."""
        dialog._warehouse_input.clear()
        # _on_create checks and returns early with error label
        # We verify it does NOT emit the signal
        signals: list[dict] = []
        dialog.task_created.connect(lambda d: signals.append(d))
        dialog._on_create()
        # The method should return before emitting when warehouse is empty
        # (self._error_label is shown instead)
        assert len(signals) == 0
        assert dialog._error_label.isVisible()

    def test_dialog_emits_signal_with_valid_data(self, dialog: _CreateTaskDialog) -> None:
        """Filling valid data should emit task_created signal with waypoints."""
        assert dialog._warehouse_input.currentText() == "粮仓D区-01号"
        # Set depth via combobox to 3.5m
        idx = dialog._depth_inputs[0].findText("3.5 米")
        if idx >= 0:
            dialog._depth_inputs[0].setCurrentIndex(idx)

        # Set waypoints: update the default item text, add a second one
        dialog._waypoint_list.clear()
        dialog._waypoint_list.addItem("点位1: X=10.0  Y=20.0")
        dialog._waypoint_list.addItem("点位2: X=30.0  Y=40.0")

        signals: list[dict] = []
        dialog.task_created.connect(lambda d: signals.append(d))

        dialog._on_create()
        assert len(signals) == 1
        data = signals[0]
        assert data["warehouse"] == "粮仓D区-01号"
        assert data["depth_list"] == [3.5]
        assert data["skip_mapping"] is False
        waypoints = data["waypoints"]
        assert len(waypoints) == 2
        assert waypoints[0] == {"x": 10.0, "y": 20.0}
        assert waypoints[1] == {"x": 30.0, "y": 40.0}
        assert "created" in data

    def test_dialog_disables_creation_without_map(
        self,
        qapp: QApplication,  # noqa: ARG002
        monkeypatch: pytest.MonkeyPatch,
        tmp_path,
    ) -> None:
        monkeypatch.setattr(task_list_module, "PCD_DIR", str(tmp_path))
        monkeypatch.delenv("GRAIN_SAMPLING_UI_SKIP_MAPPING", raising=False)

        dialog = _CreateTaskDialog()

        assert dialog._warehouse_input.count() == 0
        assert not dialog._create_btn.isEnabled()
        assert dialog._error_label.text() == "暂无地图，请先建图"

    def test_commissioning_task_skips_map_and_uses_one_depth(
        self,
        qapp: QApplication,  # noqa: ARG002
        monkeypatch: pytest.MonkeyPatch,
        tmp_path,
    ) -> None:
        monkeypatch.setattr(task_list_module, "PCD_DIR", str(tmp_path))
        monkeypatch.setenv("GRAIN_SAMPLING_UI_SKIP_MAPPING", "1")
        dialog = _CreateTaskDialog()

        assert dialog._warehouse_input.currentText() == SKIP_MAPPING_WAREHOUSE
        assert dialog._create_btn.isEnabled()

        signals: list[dict] = []
        dialog.task_created.connect(lambda data: signals.append(data))
        dialog._on_create()

        assert len(signals) == 1
        assert signals[0]["skip_mapping"] is True
        assert signals[0]["depth_list"] == [2.0]

    def test_tasklist_page_task_created_flow(self, page: TaskListPage) -> None:
        """Creating a task from task list page should add it to the list."""
        # page fixture is from TestTaskListPage — we recreate it here
        page = TaskListPage()
        initial_count = len(page._all_tasks)

        task_data = {
            "warehouse": "粮仓D区-01号",
            "depth": 3.5,
            "coord_x": 10.0,
            "coord_y": 20.0,
            "created": "2025-07-10 15:00",
        }

        created: list[dict] = []
        page.task_created.connect(lambda d: created.append(d))
        page._on_task_created(task_data)

        assert len(created) == 1
        assert len(page._all_tasks) == initial_count + 1
        new_task = page._all_tasks[-1]
        assert new_task["warehouse"] == "粮仓D区-01号"
        assert new_task["status"] == "未完成"

    def test_sample_tasks_structure(self, page: TaskListPage) -> None:
        """SAMPLE_TASKS should have correct structure."""
        assert len(SAMPLE_TASKS) == 5
        required_keys = {"id", "warehouse", "status", "created"}
        for task in SAMPLE_TASKS:
            assert required_keys.issubset(task.keys())
            assert task["status"] in ("未完成", "进行中", "已完成")

    # ── API v2 pinzhong (品种) adaptation ──────────────────────────────

    def _make_cloud_order(self, pinzhong: str = "小麦", pinzhong_code: str = "XM") -> OrderInfo:
        return OrderInfo(
            order_id="C-001",
            aojian_id=1,
            aojian="廒间1号",
            depth_list=[2.0, 3.0],
            jiance=[1, 2],
            points=[{"x": 1.0, "y": 2.0}],
            pinzhong=pinzhong,
            pinzhong_code=pinzhong_code,
        )

    @staticmethod
    def _card_detail_text(card) -> str:
        from PySide2.QtWidgets import QLabel

        for child in card.findChildren(QLabel):
            if child.objectName() == "task_line3":
                return child.text()
        return ""

    def test_cloud_order_shows_pinzhong_in_detail(self, page: TaskListPage) -> None:
        """Cloud order with pinzhong should render 品种 in the card detail."""
        page._cloud_orders = [self._make_cloud_order()]
        page._selected_cloud_index = -1
        page._refresh_cloud_list()
        assert len(page._cloud_cards) == 1
        detail = self._card_detail_text(page._cloud_cards[0])
        assert "小麦" in detail

    def test_cloud_order_without_pinzhong_omits_pinzhong(self, page: TaskListPage) -> None:
        """Cloud order without pinzhong should not render 品种 prefix."""
        order = self._make_cloud_order(pinzhong="", pinzhong_code="")
        page._cloud_orders = [order]
        page._selected_cloud_index = -1
        page._refresh_cloud_list()
        assert len(page._cloud_cards) == 1
        detail = self._card_detail_text(page._cloud_cards[0])
        assert "品种" not in detail

    def test_accept_uses_cloud_pinzhong_no_dialog(self, page: TaskListPage) -> None:
        """Accepting a cloud order uses order.pinzhong, no manual dialog."""
        order = self._make_cloud_order()
        page._cloud_orders = [order]
        page._selected_cloud_index = 0

        payloads: list[dict] = []
        page.task_selected.connect(lambda d: payloads.append(d))

        # Trigger accept-success directly (bypass network)
        page._on_accept_success(order)

        assert len(payloads) == 1
        payload = payloads[0]
        assert payload["grain_type"] == "小麦"
        assert payload["pinzhong"] == "小麦"
        assert payload["pinzhong_code"] == "XM"
        assert payload["source"] == "cloud"

    def test_accept_falls_back_to_default_when_pinzhong_empty(self, page: TaskListPage) -> None:
        """Cloud order without pinzhong falls back to 稻谷 grain_type."""
        order = self._make_cloud_order(pinzhong="", pinzhong_code="")
        page._cloud_orders = [order]
        page._selected_cloud_index = 0

        payloads: list[dict] = []
        page.task_selected.connect(lambda d: payloads.append(d))
        page._on_accept_success(order)

        assert len(payloads) == 1
        assert payloads[0]["grain_type"] == "稻谷"
