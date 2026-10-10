"""Hardware-free regression checks for uncertain relative motion completion."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from grain_sampling_workflow.orchestrator import WorkflowOrchestrator
from grain_sampling_workflow.mechanism_node import MechanismNode


class NoReplayTests(unittest.TestCase):
    def test_workflow_stops_without_replaying_relative_motion(self):
        for action in ("press", "lift", "move_lift"):
            for failure in (False, RuntimeError("moved but completion uncertain")):
                with self.subTest(action=action, failure=failure):
                    orch = WorkflowOrchestrator.__new__(WorkflowOrchestrator)
                    orch._fsm = SimpleNamespace(is_running=True)
                    orch._stop_fsm = MagicMock()
                    call = MagicMock(return_value=False)
                    if isinstance(failure, Exception):
                        call.side_effect = failure
                    self.assertFalse(orch._call_mechanism(action, call))
                    call.assert_called_once()
                    orch._stop_fsm.assert_called_once()

    def test_service_does_not_replay_on_usb_failure(self):
        node = MechanismNode.__new__(MechanismNode)
        node._x2p_rpm = 30
        node._ensure_lift_connected = MagicMock()
        node._wait_lift_device_change = MagicMock(return_value=True)
        node._controller = MagicMock()
        node._controller.move_lift.side_effect = OSError("USB lost after movement")
        response = node._handle_move_lift(SimpleNamespace(direction="up", distance_cm=20))
        self.assertFalse(response.success)
        node._controller.move_lift.assert_called_once()
        node._wait_lift_device_change.assert_not_called()


if __name__ == "__main__":
    unittest.main()
