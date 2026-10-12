#!/usr/bin/env python3
"""Check the existing Lenovo navigation session without commanding motion."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "3588/src"), str(ROOT / "3588"), str(ROOT / "LENOVO/src")]

import grain_sampling_workflow

grain_sampling_workflow.__path__.append(str(ROOT / "LENOVO/src/grain_sampling_workflow"))
from grain_sampling_workflow.navigation_session import check_session, format_report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "LENOVO/config/local_robot.json")
    parser.add_argument("--require-local", action="store_true", help="also require local navigation mode")
    args = parser.parse_args()
    try:
        report = check_session(str(args.config), require_local=args.require_local)
    except Exception as exc:
        print(f"BLOCKED: {exc}", file=sys.stderr)
        return 2
    print(format_report(report))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
