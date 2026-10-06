#!/usr/bin/env python3
"""在 Docker 內使用 PX4 pyulog，查核包含失效機的三機位置。

@file analyze_small_field_px4_logs.py
@brief 共同 UTC 時間軸下的場域與機間距離離線驗收
@author Taiphoon Agent Team
@date 2026-10-06
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from pyulog import ULog


def analyze(case):
    events = [json.loads(line) for line in (case / "monitor.jsonl").read_text().splitlines()
              if line.startswith("{")]
    reference = next(event for event in events if event["event"] == "field_reference")
    result = next(event for event in events if event["event"] == "result")
    takeoff = next(event for event in events if event.get("command") == "TAKEOFF 2")
    start, end = takeoff["time_s"], result["time_s"]
    grid = np.arange(start, end, 0.05)
    report = {"live_result": result, "sample_period_s": 0.05,
              "position_source": "PX4 vehicle_global_position estimator",
              "physical_contact_simulation": False, "vehicles": {}}
    points = []
    valid_samples = np.ones(len(grid), dtype=bool)
    complete = len(grid) > 0
    for agent in (1, 2, 3):
        paths = sorted((case / f"drone{agent}").rglob("*.ulg"))
        if not paths:
            raise ValueError(f"missing_ulog_agent_{agent}")
        log = ULog(str(paths[-1]))
        boot = log.msg_info_dict.get("boot_time_utc_us", 0) / 1e6
        if not boot:
            raise ValueError(f"missing_common_clock_agent_{agent}")
        data = next(item.data for item in log.data_list
                    if item.name == "vehicle_global_position")
        times = boot + data["timestamp"] / 1e6
        scale = 111319.49079327358
        lat, lon, alt = reference["reference"]
        positions = np.column_stack(((data["lat"] - lat) * scale,
                                    (data["lon"] - lon) * scale * math.cos(math.radians(lat)),
                                    data["alt"] - alt))
        gaps = np.diff(times)
        covered = (times[0] <= start and times[-1] >= end - 0.25 and
                   np.max(gaps) <= 0.25 and np.isfinite(positions).all())
        complete = complete and covered
        valid_samples &= (grid >= times[0]) & (grid <= times[-1])
        status = next(item.data for item in log.data_list if item.name == "vehicle_status")
        landed = next(item.data for item in log.data_list if item.name == "vehicle_land_detected")
        report["vehicles"][str(agent)] = {
            "log": str(paths[-1]), "coverage_complete": bool(covered),
            "first_time_s": float(times[0]), "last_time_s": float(times[-1]),
            "max_sample_gap_s": float(np.max(gaps)),
            "last_arming_state": int(status["arming_state"][-1]),
            "last_nav_state": int(status["nav_state"][-1]),
            "last_landed": bool(landed["landed"][-1]),
            "parameters": {key: log.initial_parameters.get(key) for key in
                           ("COM_OBL_RC_ACT", "COM_OF_LOSS_T", "RTL_RETURN_ALT", "RTL_DESCEND_ALT")}}
        points.append(np.column_stack([np.interp(grid, times, positions[:, axis])
                                       for axis in range(3)]))
    grid = grid[valid_samples]
    points = [point[valid_samples] for point in points]
    if len(grid) == 0:
        raise ValueError("no_common_position_log_interval")
    report["measured_end_time_s"] = float(grid[-1])
    pairs = {}
    for first, second in ((0, 1), (0, 2), (1, 2)):
        distances = np.linalg.norm(points[first] - points[second], axis=1)
        index = int(np.argmin(distances))
        pairs[f"{first + 1}-{second + 1}"] = {
            "min_distance_m": float(distances[index]), "time_s": float(grid[index])}
    report["pairs"] = pairs
    report["max_height_m"] = float(max(np.max(point[:, 2]) for point in points))
    report["max_horizontal_m"] = float(max(np.max(np.abs(point[:, :2])) for point in points))
    violations = (min(pair["min_distance_m"] for pair in pairs.values()) < 2.0 or
                  report["max_height_m"] > 8.0 or report["max_horizontal_m"] > 4.5)
    report["status"] = ("FAIL" if violations or not result["pass_live"] else
                        "PASS_SAMPLED" if complete else "INCONCLUSIVE")
    report["coverage_complete"] = bool(complete)
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("case", type=Path)
    args = parser.parse_args()
    try:
        report = analyze(args.case)
    except (ValueError, StopIteration, OSError) as error:
        report = {"status": "INCONCLUSIVE", "error": str(error)}
    (args.case / "px4-analysis.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["status"] == "PASS_SAMPLED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
