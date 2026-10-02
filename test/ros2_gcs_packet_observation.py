#!/usr/bin/env python3
"""
@file ros2_gcs_packet_observation.py
@brief 在 Docker 開發容器觀測指定 GCS 的 UDP／RTPS 封包標頭，不記錄資料內容
@author Taiphoon Agent Team
@date 2026-10-02
"""
import collections
import json
from pathlib import Path
import socket
import struct
import sys
import time


def main():
    """只統計 192.168.0.108 的 RTPS 方向與端點，沒有回包或修改路由。"""
    counts = collections.Counter()
    with socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0800)) as capture:
        capture.settimeout(1)
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            try:
                packet = capture.recv(65535)
            except socket.timeout:
                continue
            if len(packet) < 42 or packet[23] != 17:
                continue
            header_bytes = (packet[14] & 15) * 4
            udp = 14 + header_bytes
            if len(packet) < udp + 12:
                continue
            source = socket.inet_ntoa(packet[26:30])
            target = socket.inet_ntoa(packet[30:34])
            if '192.168.0.108' not in (source, target) or packet[udp + 8:udp + 12] != b'RTPS':
                continue
            source_port, target_port = struct.unpack('!HH', packet[udp:udp + 4])
            counts[f'{source}:{source_port} -> {target}:{target_port}'] += 1
    result = {'gcs': '192.168.0.108', 'observed_rtps_packets': sum(counts.values()),
              'outgoing_rtps_packets': sum(count for endpoint, count in counts.items()
                                           if ' -> 192.168.0.108:' in endpoint),
              'incoming_rtps_packets': sum(count for endpoint, count in counts.items()
                                           if endpoint.startswith('192.168.0.108:')),
              'endpoints': dict(counts),
              'note': '封包觀測不能取代遠端 GCS 畫面與訂閱驗收'}
    if len(sys.argv) > 1:
        Path(sys.argv[1]).write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({key: value for key, value in result.items() if key != 'endpoints'},
                     ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
