import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "3588" / "src"), str(ROOT / "3588"), str(ROOT / "LENOVO" / "src")]
