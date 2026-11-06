#!/usr/bin/env python3
"""驗證邊界驗收器不把 B-spline 控制點誤當成飛行位置。

@file small_field_monitor_test.py
@brief 控制點凸包與實際 cubic 曲線邊界回歸
@author Taiphoon Agent Team
@date 2026-10-06
"""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

spec = importlib.util.spec_from_file_location(
    "small_field_monitor", Path(__file__).parents[1] /
    "AutoTest/scripts/small_field_three_agent_monitor.py")
monitor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(monitor)


class CurveBoundaryTest(unittest.TestCase):
    def validate(self, east, half_long_m=4.5, half_short_m=4.5, bearing_deg=0.0):
        events = []
        observer = SimpleNamespace(reference=[0, 0, 0], failure=None,
                                   field_half_long_m=half_long_m, field_half_short_m=half_short_m,
                                   field_bearing_rad=math.radians(bearing_deg),
                                   project=lambda point: point,
                                   emit=lambda event, **values: events.append((event, values)))
        observer.field_frame = lambda local: monitor.SmallFieldMonitor.field_frame(observer, local)
        message = SimpleNamespace(valid=True, order=3, agent_index=1, trajectory_id=42,
                                  control_points=[value for point in east for value in (0, point, 2)])
        monitor.SmallFieldMonitor.trajectory_cb(observer, message)
        return observer.failure, events

    def test_control_point_outside_but_curve_inside(self):
        # 單一內部控制點 6 m；cubic basis 的峰值 2/3，實際曲線最大為 4 m。
        failure, events = self.validate([0, 0, 0, 6, 0, 0, 0])
        self.assertIsNone(failure)
        self.assertEqual(events, [])

    def test_actual_curve_outside_records_evidence(self):
        failure, events = self.validate([5] * 7)
        self.assertEqual(failure, "trajectory_outside_field_agent_1")
        self.assertAlmostEqual(events[0][1]["value_m"], 5)

    def test_rotated_field_uses_long_axis(self):
        # 長軸朝東（90°）：東向 5 m 落在長軸半長 6 m 內；同樣軌跡在短軸半長 3 m 的未旋轉場域會超界。
        failure, events = self.validate([5] * 7, half_long_m=6.0, half_short_m=3.0, bearing_deg=90.0)
        self.assertIsNone(failure)
        self.assertEqual(events, [])
        failure, _ = self.validate([5] * 7, half_long_m=6.0, half_short_m=3.0, bearing_deg=0.0)
        self.assertEqual(failure, "trajectory_outside_field_agent_1")


if __name__ == "__main__":
    unittest.main()
