#!/usr/bin/env python3
"""Real insertion test through production services. Never run alongside UI tasks.
Usage: python3 scripts/sampling_press.py target_cm [segment_cm]
Tune reciprocation in sampling_params.py and restart the mechanism service.
"""
import argparse
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "src"))
from sampling_params import PRESS_SEGMENT_CM
from grain_sampling_workflow.mechanism_config import get_grain_params


def run_segments(target_cm, segment_cm, action, move):
    if not all(math.isfinite(v) and v > 0 for v in (target_cm, segment_cm)):
        raise ValueError("Distances must be finite and positive")
    total = 0.0
    while total < target_cm - 1e-9:
        distance = min(segment_cm, target_cm - total)
        action("clamp")
        move("down_cycle", distance)
        action("unclamp")
        move("return", distance)
        action("clamp")
        total += distance
        print(f"Completed {total:g}/{target_cm:g} cm", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("target_cm", type=float)
    parser.add_argument("segment_cm", type=float, nargs="?", default=PRESS_SEGMENT_CM)
    args = parser.parse_args()
    if not all(math.isfinite(v) and v > 0 for v in (args.target_cm, args.segment_cm)):
        parser.error("Distances must be finite and positive")
    import rospy
    from std_srvs.srv import Trigger
    from mechanism_node.srv import MoveLift

    rospy.init_node("sampling_press", anonymous=True)
    proxies = {}
    for name, kind in (("clamp", Trigger), ("unclamp", Trigger),
                       ("emergency_stop", Trigger), ("move_lift", MoveLift)):
        path = "/mechanism/" + name
        rospy.wait_for_service(path, timeout=5)
        proxies[name] = rospy.ServiceProxy(path, kind)
    if input("REAL HARDWARE: stop UI tasks, check origin and safety. Type YES: ").strip() != "YES":
        return 0

    def checked(name, **kwargs):
        response = proxies[name](**kwargs)
        if not response.success:
            raise RuntimeError(f"{name}: {response.message}")

    # Conservatively wait for the longest configured clamp duration.
    from sampling_params import GRAIN_MECHANISM_CONFIG
    configs = [get_grain_params("")] + list(GRAIN_MECHANISM_CONFIG.values())

    def action(name):
        checked(name)
        rospy.sleep(max(p[name + "_duration"] for p in configs) + 0.5)

    try:
        run_segments(args.target_cm, args.segment_cm, action,
                     lambda direction, distance: checked(
                         "move_lift", direction=direction, distance_cm=distance))
    except (Exception, KeyboardInterrupt):
        try:
            checked("emergency_stop")
        except Exception as error:
            print(f"Software stop failed; use hardware emergency stop: {error}", file=sys.stderr)
        raise
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
