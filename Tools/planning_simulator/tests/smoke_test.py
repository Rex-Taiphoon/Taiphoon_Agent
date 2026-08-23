#!/usr/bin/env python3
"""檢查離線 engine 與提供的 JSON 情境是否仍可正常串接。"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path


TOOL_ROOT = Path(__file__).resolve().parents[1]
SERVER_ROOT = TOOL_ROOT / "server"
SCENARIOS = ("open_space.json", "fixed_obstacle_peer.json")


def assert_vector_close(actual: list[float], expected: list[float], name: str) -> None:
    """確認三維向量符合 Local Goal 的預期。"""
    if len(actual) != 3 or any(abs(a - e) > 1.0e-6 for a, e in zip(actual, expected)):
        raise AssertionError(f"{name}: expected {expected}, got {actual}")


def main() -> int:
    parser = argparse.ArgumentParser(description="planning_simulator smoke test")
    parser.add_argument("--engine", required=True, type=Path)
    args = parser.parse_args()

    sys.path.insert(0, str(SERVER_ROOT))
    import server  # pylint: disable=import-outside-toplevel

    for scenario_name in SCENARIOS:
        scenario_path = TOOL_ROOT / "scenarios" / scenario_name
        scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
        completed = subprocess.run(
            [str(args.engine), "--stdin"],
            input=server.scenario_to_protocol(scenario),
            text=True,
            capture_output=True,
            check=False,
            timeout=10.0,
        )
        if completed.returncode != 0:
            raise RuntimeError(f"{scenario_name}: engine exit {completed.returncode}: {completed.stderr}")
        result = json.loads(completed.stdout)
        if not result.get("ok"):
            raise RuntimeError(f"{scenario_name}: engine returned error: {result}")
        if not result["planner"]["is_valid"] or not result["safety"]["is_safe"]:
            raise RuntimeError(f"{scenario_name}: planner/safety check failed: {result}")
        if not result["samples"]:
            raise RuntimeError(f"{scenario_name}: planner returned no samples")
        final_goal = scenario["self"]["goal_ned"]
        local_goal = result["input"]["local_goal"]
        assert_vector_close(local_goal, [10.0, 0.0, -5.0], f"{scenario_name}: local goal")
        assert_vector_close(result["control_points"][-1], local_goal,
                            f"{scenario_name}: planned endpoint")
        if local_goal == final_goal:
            raise AssertionError(
                f"{scenario_name}: long-distance Final Target bypassed Local Goal")
        print(
            f"{scenario_name}: termination={result['planner']['termination']} "
            f"local_goal={local_goal} samples={len(result['samples'])} safety=PASS"
        )

    short_scenario = json.loads(
        (TOOL_ROOT / "scenarios" / "open_space.json").read_text(encoding="utf-8"))
    short_scenario["self"]["goal_ned"] = [5.0, 0.0, -5.0]
    completed = subprocess.run(
        [str(args.engine), "--stdin"],
        input=server.scenario_to_protocol(short_scenario),
        text=True,
        capture_output=True,
        check=False,
        timeout=10.0,
    )
    if completed.returncode != 0:
        raise RuntimeError(f"short goal: engine exit {completed.returncode}: {completed.stderr}")
    result = json.loads(completed.stdout)
    if not result.get("ok"):
        raise RuntimeError(f"short goal: engine returned error: {result}")
    assert_vector_close(result["input"]["local_goal"], [5.0, 0.0, -5.0],
                        "short goal: final target should remain local goal")
    assert_vector_close(result["control_points"][-1], result["input"]["local_goal"],
                        "short goal: planned endpoint")
    print("short goal: final target retained as local goal")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
