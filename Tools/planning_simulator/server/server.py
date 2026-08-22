#!/usr/bin/env python3
"""Taiphoon Agent 離線規劃模擬器的本機 HTTP 服務。

此服務只使用 Python 標準函式庫。瀏覽器送出的 JSON 情境會被轉成
planning_simulator 的 stdin 協定，再將 C++ planner 的 JSON 結果回傳。
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import unquote


ROOT = Path(__file__).resolve().parents[1]
WEB_ROOT = ROOT / "web"


def finite_number(value: Any, name: str) -> float:
    """驗證一個可傳入 C++ planner 的有限浮點值。"""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} 必須是數值")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} 必須是有限值")
    return number


def vector(value: Any, name: str) -> list[float]:
    """驗證三維向量。"""
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{name} 必須是三個數值")
    return [finite_number(item, f"{name}[{index}]")
            for index, item in enumerate(value)]


def protocol_line_vector(key: str, values: list[float]) -> str:
    """建立 engine 的三維向量輸入行。"""
    return f"{key} {values[0]:.17g} {values[1]:.17g} {values[2]:.17g}"


def scenario_to_protocol(scenario: dict[str, Any]) -> str:
    """把瀏覽器情境轉成無 ROS2 的 engine stdin 協定。"""
    self_state = scenario.get("self", {})
    planner = scenario.get("planner", {})
    if not isinstance(self_state, dict) or not isinstance(planner, dict):
        raise ValueError("self 與 planner 必須是 JSON object")
    obstacles = scenario.get("obstacles", [])
    peers = scenario.get("peers", [])
    if not isinstance(obstacles, list) or not isinstance(peers, list):
        raise ValueError("obstacles 與 peers 必須是 JSON array")
    lines = [
        protocol_line_vector(
            "self_position", vector(self_state.get("position_ned", [0, 0, -5]), "self.position_ned")),
        protocol_line_vector(
            "self_velocity", vector(self_state.get("velocity_ned", [0, 0, 0]), "self.velocity_ned")),
        protocol_line_vector(
            "self_acceleration", vector(self_state.get("acceleration_ned", [0, 0, 0]), "self.acceleration_ned")),
        protocol_line_vector(
            "goal", vector(self_state.get("goal_ned", [10, 0, -5]), "self.goal_ned")),
        f"path_constraint {scenario.get('path_constraint', 'free_3d')}",
        f"sample_spacing_m {finite_number(scenario.get('sample_spacing_m', 0.5), 'sample_spacing_m'):.17g}",
    ]

    allowed_planner = {
        "control_point_count": int,
        "knot_interval_s": float,
        "sample_interval_s": float,
        "obstacle_clearance_m": float,
        "peer_collision_min_distance_m": float,
        "max_velocity_mps": float,
        "max_acceleration_mps2": float,
        "max_jerk_mps3": float,
        "optimizer_max_iterations": int,
        "optimizer_time_budget_ms": float,
        "candidate_offset_m": float,
    }
    for key, kind in allowed_planner.items():
        if key not in planner:
            continue
        value = planner[key]
        if kind is int:
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f"planner.{key} 必須是整數")
            checked = value
            if key == "control_point_count" and not 4 <= checked <= 200:
                raise ValueError("planner.control_point_count 必須介於 4 與 200")
            if key == "optimizer_max_iterations" and not 1 <= checked <= 10000:
                raise ValueError("planner.optimizer_max_iterations 必須介於 1 與 10000")
        else:
            checked = finite_number(value, f"planner.{key}")
            lower_bound = 0.0
            if key in {
                "knot_interval_s", "sample_interval_s", "max_velocity_mps",
                "max_acceleration_mps2", "max_jerk_mps3",
                "optimizer_time_budget_ms",
            }:
                lower_bound = 1.0e-9
            if checked < lower_bound:
                raise ValueError(f"planner.{key} 必須大於或等於 {lower_bound:g}")
        lines.append(f"planner {key} {checked}")

    sample_spacing = finite_number(
        scenario.get("sample_spacing_m", 0.5), "sample_spacing_m")
    if sample_spacing <= 0.0:
        raise ValueError("sample_spacing_m 必須大於 0")

    for index, obstacle in enumerate(obstacles):
        if not isinstance(obstacle, dict):
            raise ValueError(f"obstacles[{index}] 必須是物件")
        obstacle_type = obstacle.get("type")
        center = vector(obstacle.get("center_ned"), f"obstacles[{index}].center_ned")
        if obstacle_type in ("box", "wall"):
            size = vector(obstacle.get("size_m"), f"obstacles[{index}].size_m")
            if any(item <= 0.0 for item in size):
                raise ValueError(f"obstacles[{index}].size_m 必須全部大於 0")
            lines.append(protocol_line_vector(f"obstacle {obstacle_type}", center) +
                         f" {size[0]:.17g} {size[1]:.17g} {size[2]:.17g}")
        elif obstacle_type == "cylinder":
            radius = finite_number(obstacle.get("radius_m"), f"obstacles[{index}].radius_m")
            height = finite_number(obstacle.get("height_m"), f"obstacles[{index}].height_m")
            if radius <= 0.0 or height <= 0.0:
                raise ValueError(f"obstacles[{index}] 的半徑與高度必須大於 0")
            lines.append(protocol_line_vector("obstacle cylinder", center) +
                         f" {radius:.17g} {height:.17g}")
        else:
            raise ValueError(f"obstacles[{index}].type 不支援")

    for index, peer in enumerate(peers):
        if not isinstance(peer, dict):
            raise ValueError(f"peers[{index}] 必須是物件")
        agent_id = peer.get("agent_id")
        if isinstance(agent_id, bool) or not isinstance(agent_id, int) or agent_id <= 0:
            raise ValueError(f"peers[{index}].agent_id 必須是正整數")
        mode = peer.get("mode", "hold")
        if mode not in ("hold", "move"):
            raise ValueError(f"peers[{index}].mode 不支援")
        start = vector(peer.get("start_ned"), f"peers[{index}].start_ned")
        line = f"peer {agent_id} {mode} {start[0]:.17g} {start[1]:.17g} {start[2]:.17g}"
        if mode == "move":
            goal = vector(peer.get("goal_ned"), f"peers[{index}].goal_ned")
            line += f" {goal[0]:.17g} {goal[1]:.17g} {goal[2]:.17g}"
        lines.append(line)

    lines.append("end")
    return "\n".join(lines) + "\n"


class Handler(BaseHTTPRequestHandler):
    """提供靜態頁面與單一規劃 API。"""

    engine_path: Path

    def _send_bytes(self, status: int, content_type: str, body: bytes) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self._send_bytes(status, "application/json; charset=utf-8", body)

    def do_GET(self) -> None:  # noqa: N802
        """傳送 UI 靜態檔案。"""
        request_path = unquote(self.path.split("?", 1)[0])
        relative = "index.html" if request_path in ("", "/") else request_path.lstrip("/")
        candidate = (WEB_ROOT / relative).resolve()
        if WEB_ROOT not in candidate.parents or not candidate.is_file():
            self._send_json(404, {"ok": False, "error": "找不到資源"})
            return
        content_type = "text/plain; charset=utf-8"
        if candidate.suffix == ".html":
            content_type = "text/html; charset=utf-8"
        elif candidate.suffix == ".js":
            content_type = "text/javascript; charset=utf-8"
        elif candidate.suffix == ".css":
            content_type = "text/css; charset=utf-8"
        self._send_bytes(200, content_type, candidate.read_bytes())

    def do_POST(self) -> None:  # noqa: N802
        """執行一次離線規劃。"""
        if self.path != "/plan":
            self._send_json(404, {"ok": False, "error": "找不到 API"})
            return
        try:
            content_length = int(self.headers.get("Content-Length", "0"))
            if content_length <= 0 or content_length > 1_000_000:
                raise ValueError("請求大小不合法")
            scenario = json.loads(self.rfile.read(content_length).decode("utf-8"))
            if not isinstance(scenario, dict):
                raise ValueError("情境必須是 JSON object")
            protocol = scenario_to_protocol(scenario)
            completed = subprocess.run(
                [str(self.engine_path), "--stdin"],
                input=protocol,
                text=True,
                capture_output=True,
                check=False,
                timeout=10.0,
            )
            if not completed.stdout.strip():
                raise RuntimeError(completed.stderr.strip() or "planner 沒有輸出")
            result = json.loads(completed.stdout)
            status = 200 if completed.returncode == 0 else 422
            self._send_json(status, result)
        except subprocess.TimeoutExpired:
            self._send_json(504, {"ok": False, "error": "planner 執行逾時"})
        except (OSError, ValueError, json.JSONDecodeError, RuntimeError) as error:
            self._send_json(400, {"ok": False, "error": str(error)})

    def log_message(self, format_string: str, *args: Any) -> None:
        """保留簡潔的本機服務日誌。"""
        print(f"[planning-simulator] {format_string % args}")


def main() -> int:
    """啟動本機服務。"""
    parser = argparse.ArgumentParser(description="Taiphoon 離線路徑規劃 UI")
    parser.add_argument("--engine", required=True, type=Path,
                        help="planning_simulator executable 路徑")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8765, type=int)
    args = parser.parse_args()
    engine = args.engine.resolve()
    if not engine.is_file() or not engine.stat().st_mode & 0o111:
        parser.error(f"engine 不存在或不可執行：{engine}")
    Handler.engine_path = engine
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"[planning-simulator] 開啟 http://{args.host}:{args.port}")
    print(f"[planning-simulator] engine={engine}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[planning-simulator] 已停止")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
