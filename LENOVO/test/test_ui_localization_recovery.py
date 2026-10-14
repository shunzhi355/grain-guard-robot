"""Recovery UI stays responsive and never reinitializes a running estimator."""
import json
import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import pytest
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

@pytest.fixture
def window(monkeypatch):
    import importlib.util
    spec=importlib.util.spec_from_file_location("recovery_ui_launcher",Path(__file__).resolve().parents[2]/"3588/scripts/start_ui.py")
    launcher=importlib.util.module_from_spec(spec);spec.loader.exec_module(launcher)
    launcher.load_qt()
    from PySide2.QtWidgets import QApplication
    from grain_sampling_ui.pages.main_page import CameraWidget
    from grain_sampling_ui.main import MainWindow
    app=QApplication.instance() or QApplication([])
    monkeypatch.delenv("GRAIN_LOCAL_RUNTIME",raising=False)
    monkeypatch.setattr(CameraWidget,"_start",lambda *a:None)
    w=MainWindow()
    yield w,app
    monkeypatch.delenv("GRAIN_LOCAL_RUNTIME",raising=False)
    w.close();app.processEvents()


def test_recovery_displays_current_state_without_clearing_chassis_alarm(window):
    w,app=window
    w._on_localization_status({"state":"BLOCKED","reason":"stale_or_future_pose"})
    assert w._map_page._btn_localization.isEnabled()
    assert "stale_or_future_pose" in w._map_page._slice_note.text()
    w._on_localization_status({"state":"VALIDATING_STATIC","map_id":"active-map"})
    assert not w._map_page._btn_localization.isEnabled()
    assert "60秒" in w._map_page._slice_note.text()
    w._alarm_bar.set_alarm("底盘锁存，请复位")
    w._on_localization_status({"state":"READY","map_id":"active-map"})
    assert "active-map" in w._map_page._slice_note.text()
    assert w._alarm_bar._alarm_label.text()=="底盘锁存，请复位"
    assert not w._map_page._btn_navigate.isEnabled()  # READY alone is not motion admission
    w._on_localization_status({"state":"WAIT_INITIAL_POSE","map_id":"active-map"})
    assert w._map_page._btn_localization.isEnabled()


@pytest.mark.parametrize("state",["VALIDATING_STATIC","READY"])
def test_repeat_click_does_not_write_initial_pose_or_show_confirmation(window,monkeypatch,state):
    from PySide2.QtWidgets import QMessageBox
    w,app=window
    monkeypatch.setenv("GRAIN_LOCAL_RUNTIME","1")
    calls=[]
    w._slam_bridge=SimpleNamespace(confirm_map_origin=lambda:calls.append("WRITE"))
    w._localization_status={"state":state}
    w._poll_localization_status=lambda:calls.append("READ")
    monkeypatch.setattr(QMessageBox,"question",lambda *a:pytest.fail("running estimator must not prompt again"))
    w._on_start_localization()
    assert calls==["READ"]


def test_docker_status_wait_does_not_block_qt_event_loop(window):
    w,app=window
    entered,released=threading.Event(),threading.Event()
    def reader():
        entered.set(); released.wait(2)
        return {"state":"READY","map_id":"active-map"}
    w._slam_bridge=SimpleNamespace(read_localization_startup_status=reader)
    try:
        w._poll_localization_status()
        assert entered.wait(1)
        w._map_page._slice_note.setText("UI is responsive")
        app.processEvents()
        assert w._map_page._slice_note.text()=="UI is responsive"
        w._poll_localization_status()  # no overlapping Docker probes
    finally:
        released.set()
    until=time.monotonic()+2
    while w._localization_status_loading and time.monotonic()<until:
        app.processEvents();time.sleep(.005)
    assert w._localization_status.get("state")=="READY"


@pytest.mark.parametrize("state",["VALIDATING_STATIC","READY"])
def test_bridge_repeat_confirmation_is_idempotent(monkeypatch,state):
    from grain_sampling_local.slam import LocalSlamBridge
    bridge=LocalSlamBridge()
    bridge.read_localization_startup_status=lambda:{"state":state}
    monkeypatch.setattr("grain_sampling_local.slam.subprocess.run",lambda *a,**kw:
        pytest.fail("must not rewrite initialization during validation or READY"))
    assert bridge.confirm_map_origin()


def active_preview_data():
    from PySide2.QtGui import QImage,QColor
    image=QImage(100,100,QImage.Format.Format_RGB32);image.fill(QColor("white"))
    return {"image":image,"world_bounds":[-1.,2.,-1.,2.],"map_id":"active-map","map_version":"r1",
            "frame":"camera_init","manifest_sha256":"a"*64,"active_map_match":True,
            "candidate":False,"z_slice":[.4,2.],"pcd_sha256":"b"*64}


def bind_runtime_map(tmp_path,monkeypatch,data,**overrides):
    """Point the UI at a runtime map binding for the given preview."""
    pointer=tmp_path/"state"/"active_map_preview.json"
    pointer.parent.mkdir(parents=True,exist_ok=True)
    binding={"map_id":data["map_id"],"map_version":data["map_version"],"frame":data["frame"],
             "pcd_sha256":data.get("pcd_sha256",""),"package_manifest_sha256":data["manifest_sha256"]}
    binding.update(overrides)
    (pointer.parent/"active_runtime_map.json").write_text(json.dumps(binding))
    monkeypatch.setenv("GRAIN_SAVED_MAP_PREVIEW",str(pointer))
    return pointer


def _select_goal_and_wait(w,app):
    w._map_page.set_selected_goal(.3,.3)
    w._map_page.telemetry.update_status({"pose":{"x_m":0.,"y_m":0.,"odom_at":time.monotonic(),"localization_valid":True}})
    app.processEvents()


def test_real_map_click_enables_button_only_after_active_map_binding(window,monkeypatch,tmp_path):
    from PySide2.QtCore import Qt,QPoint
    from PySide6.QtTest import QTest
    w,app=window;monkeypatch.setenv("GRAIN_LOCAL_RUNTIME","1")
    d=active_preview_data()
    bind_runtime_map(tmp_path,monkeypatch,d)
    w._on_localization_status({"state":"READY",**{k:d[k] for k in ("map_id","map_version","frame")}})
    w._on_saved_map_preview(d)
    w._map_page.telemetry.update_status({"pose":{"x_m":0.,"y_m":0.,"odom_at":time.monotonic(),"localization_valid":True},"motion_blocked":False})
    assert not w._map_page._btn_navigate.isEnabled()  # goal required
    w.show();w.page_manager.push("map");app.processEvents()
    point=w._map_page.map_widget._world_to_widget(.3,.3)
    QTest.mouseClick(w._map_page.map_widget,Qt.MouseButton.LeftButton,pos=QPoint(round(point.x()),round(point.y())))
    app.processEvents()
    assert w._map_page._selected_goal is not None
    assert w._map_page._btn_navigate.isEnabled()
    d2={**d,"map_id":"another-map"}
    w._on_saved_map_preview(d2)
    assert w._map_page._selected_goal is None
    assert not w._map_page._btn_navigate.isEnabled()


@pytest.mark.parametrize("field",["map_id","map_version","frame"])
def test_active_preview_must_match_live_identity(window,monkeypatch,tmp_path,field):
    w,app=window;monkeypatch.setenv("GRAIN_LOCAL_RUNTIME","1")
    d=active_preview_data()
    bind_runtime_map(tmp_path,monkeypatch,d)
    startup={"state":"READY",**{k:d[k] for k in ("map_id","map_version","frame")}}
    startup[field]="wrong"
    w._on_localization_status(startup);w._on_saved_map_preview(d)
    _select_goal_and_wait(w,app)
    assert not w._map_page._btn_navigate.isEnabled()


@pytest.mark.parametrize("overrides",[
    {"package_manifest_sha256":"c"*64},
    {"pcd_sha256":"d"*64},
    {"map_id":"another-map"},
])
def test_runtime_binding_mismatch_blocks_navigation(window,monkeypatch,tmp_path,overrides):
    """A preview that is not the map loaded into Golden must never navigate."""
    w,app=window;monkeypatch.setenv("GRAIN_LOCAL_RUNTIME","1")
    d=active_preview_data()
    bind_runtime_map(tmp_path,monkeypatch,d,**overrides)
    w._on_localization_status({"state":"READY",**{k:d[k] for k in ("map_id","map_version","frame")}})
    w._on_saved_map_preview(d)
    _select_goal_and_wait(w,app)
    assert not w._map_page._btn_navigate.isEnabled()


def test_missing_runtime_binding_blocks_navigation(window,monkeypatch,tmp_path):
    w,app=window;monkeypatch.setenv("GRAIN_LOCAL_RUNTIME","1")
    d=active_preview_data()
    monkeypatch.setenv("GRAIN_SAVED_MAP_PREVIEW",str(tmp_path/"state"/"active_map_preview.json"))
    w._on_localization_status({"state":"READY",**{k:d[k] for k in ("map_id","map_version","frame")}})
    w._on_saved_map_preview(d)
    _select_goal_and_wait(w,app)
    assert not w._map_page._btn_navigate.isEnabled()


def test_fault_reset_button_refuses_unsafe_chassis_state(window,monkeypatch):
    from PySide2.QtWidgets import QMessageBox
    w,app=window
    calls=[]

    class Bridge:
        last_error=""

        class client:
            @staticmethod
            def request(action,**values):
                return {"chassis":{"chassis_link":"online","rc_mode":"manual","rc_valid":True,
                                   "faults":0,"motion_armed":False},
                        "safety_latched":False,"recovery":{"reset_required":False}}

        @staticmethod
        def call_restore_control():
            calls.append("restore")
            return True

    monkeypatch.setattr("grain_sampling_ui.main.SamplingBridge",lambda *a,**k:Bridge())
    monkeypatch.setattr(QMessageBox,"question",lambda *a,**k:pytest.fail("must not ask to confirm"))
    assert w._fault_reset_btn.text()=="故障复位"
    w._on_clear_estop()
    assert calls==[]
    assert "复位条件不足" in w._alarm_bar._alarm_label.text()


def test_fault_reset_clears_latch_and_hands_back_control(window,monkeypatch):
    from PySide2.QtWidgets import QMessageBox
    w,app=window
    calls=[];notices=[]
    latched={"chassis":{"chassis_link":"online","rc_mode":"auto","rc_valid":True,"faults":0,
                        "motion_armed":False,"estop_latched":True},"safety_latched":True,
             "recovery":{"reset_required":True},"pose":{},"navigation":{"goal_id":"old","result":"FAILED"}}
    clean={"chassis":{"chassis_link":"online","rc_mode":"auto","rc_valid":True,"faults":0,
                      "motion_armed":False,"estop_latched":False},"safety_latched":False,
           "recovery":{"reset_required":False},"pose":{},"navigation":{"goal_id":"old","result":"FAILED"}}
    statuses=[latched,clean]

    class Bridge:
        last_error=""

        class client:
            @staticmethod
            def request(action,**values):
                return statuses.pop(0) if len(statuses)>1 else statuses[0]

        @staticmethod
        def call_restore_control():
            calls.append("restore")
            return True

    monkeypatch.setattr("grain_sampling_ui.main.SamplingBridge",lambda *a,**k:Bridge())
    monkeypatch.setattr(QMessageBox,"question",lambda *a,**k:QMessageBox.StandardButton.Yes)
    monkeypatch.setattr(QMessageBox,"information",lambda *a,**k:notices.append(a[2] if len(a)>2 else ""))
    w._on_clear_estop()
    assert calls==["restore"]                       # no goal is ever re-issued
    assert any("重新选点" in text for text in notices)
    assert "复位条件不足" not in w._alarm_bar._alarm_label.text()


def test_fault_reset_allows_navigation_state_without_a_latch(window,monkeypatch):
    """One button: no latch set still hands control back for a new goal."""
    from PySide2.QtWidgets import QMessageBox
    w,app=window
    calls=[];notices=[]
    clean={"chassis":{"chassis_link":"online","rc_mode":"auto","rc_valid":True,"faults":0,
                      "motion_armed":False,"estop_latched":False},"safety_latched":False,
           "recovery":{"reset_required":False},"pose":{},"navigation":None}

    class Bridge:
        last_error=""

        class client:
            @staticmethod
            def request(action,**values):
                return clean

        @staticmethod
        def call_restore_control():
            calls.append("restore")
            return True

    monkeypatch.setattr("grain_sampling_ui.main.SamplingBridge",lambda *a,**k:Bridge())
    monkeypatch.setattr(QMessageBox,"question",lambda *a,**k:QMessageBox.StandardButton.Yes)
    monkeypatch.setattr(QMessageBox,"information",lambda *a,**k:notices.append(a[2] if len(a)>2 else ""))
    w._on_clear_estop()
    assert calls==["restore"]
    assert any("重新选点" in text for text in notices)


def test_fault_reset_reports_unconfirmed_latch(window,monkeypatch):
    from PySide2.QtWidgets import QMessageBox
    w,app=window
    stuck={"chassis":{"chassis_link":"online","rc_mode":"auto","rc_valid":True,"faults":0,
                      "motion_armed":False,"estop_latched":True},"safety_latched":True,
           "recovery":{"reset_required":True},"pose":{},"navigation":None}

    class Bridge:
        last_error=""

        class client:
            @staticmethod
            def request(action,**values):
                return stuck

        @staticmethod
        def call_restore_control():
            return True

    monkeypatch.setattr("grain_sampling_ui.main.SamplingBridge",lambda *a,**k:Bridge())
    monkeypatch.setattr(QMessageBox,"question",lambda *a,**k:QMessageBox.StandardButton.Yes)
    monkeypatch.setattr(QMessageBox,"information",lambda *a,**k:pytest.fail("must not claim success"))
    w._on_clear_estop()
    assert "未完成" in w._alarm_bar._alarm_label.text()


def test_int_and_text_map_version_are_the_same_map(window,monkeypatch,tmp_path):
    """Manifest int version and Golden text version must not block navigation."""
    w,app=window;monkeypatch.setenv("GRAIN_LOCAL_RUNTIME","1")
    d={**active_preview_data(),"map_version":1}
    bind_runtime_map(tmp_path,monkeypatch,d)
    startup={"state":"READY","map_id":d["map_id"],"map_version":"1","frame":d["frame"]}
    w._on_localization_status(startup);w._on_saved_map_preview(d)
    _select_goal_and_wait(w,app)
    assert w._map_page._btn_navigate.isEnabled()
