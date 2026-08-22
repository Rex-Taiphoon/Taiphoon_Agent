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
        print(
            f"{scenario_name}: termination={result['planner']['termination']} "
            f"samples={len(result['samples'])} safety=PASS"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
