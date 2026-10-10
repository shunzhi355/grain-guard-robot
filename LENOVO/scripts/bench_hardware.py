#!/usr/bin/env python3
"""Simulation-only migration gate. No execute flag or physical port parameter."""
from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
# Select the complete 3588 package, not LENOVO's protocol-only namespace.
sys.path[:0] = [str(ROOT / "3588" / "src"), str(ROOT / "3588"), str(ROOT / "LENOVO" / "src")]


def main():
    parser = argparse.ArgumentParser(description="Lenovo migration SIMULATION ONLY; never opens hardware/SSH")
    parser.add_argument("--simulate", action="store_true", help="explicitly select the only available mode")
    parser.add_argument("--cycles", type=int, default=20, help="simulated round trips per normal/combined scenario (1..1000)")
    parser.add_argument("--scenario", default="all", help="all or a scenario name from the report/docs")
    parser.add_argument("--report", type=Path, required=True, help="new JSON output file; existing files are not overwritten")
    args = parser.parse_args()
    if args.report.exists():
        parser.error("report already exists; choose a new filename")
    if not 1 <= args.cycles <= 1000:
        parser.error("cycles must be in 1..1000")
    logging.basicConfig(level=logging.CRITICAL)  # injected failures are captured in JSON
    from grain_sampling_bench.runner import run_suite
    args.report.parent.mkdir(parents=True, exist_ok=True)
    report = run_suite(cycles=args.cycles, scenario=args.scenario,
                       log_path=args.report.with_suffix(".motion.log"))
    with args.report.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
    for item in report["scenarios"]:
        print(f"{'PASS' if item['passed'] else 'FAIL'} {item['name']} ({item['elapsed_s']:.3f}s)")
        if not item["passed"]:
            print(f"  {item['error']}")
    print(f"SIMULATION ONLY: physical_hardware_verified=false; decision={report['decision']}")
    print(f"Report: {args.report.resolve()}")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
