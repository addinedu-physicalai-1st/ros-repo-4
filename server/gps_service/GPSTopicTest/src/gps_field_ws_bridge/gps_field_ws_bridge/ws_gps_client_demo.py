#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Minimal WebSocket client: send PinkyGps-style JSON (no ROS required).

토픽 이름의 숫자(``/pinky_N/gps_pos``)는 JSON 필드 ``ros_main_id`` 로 덮어쓸 수 있습니다.

* ``--ros-main-id N`` 이 있으면 JSON에 ``"ros_main_id": N`` 포함.
* 없으면 환경변수 ``ROS_DOMAIN_ID`` 가 설정되어 있으면 그 정수를 ``ros_main_id`` 로 넣음.
* 둘 다 없으면 ``ros_main_id`` 키를 생략 → 서버는 서버 쪽 ``ROS_DOMAIN_ID`` 로 토픽 결정.

**토픽에 샘플 1건만 보내기** (``--loop`` 생략 = 기본):

    ros2 run gps_field_ws_bridge ws_gps_client_demo \\
        --uri ws://127.0.0.1:8765 --x 940 --y 705 --yaw 90 --ros-main-id 22

선택: ``--pinky-id 0`` (없으면 서버가 ROS_DOMAIN_ID 하위 바이트로 채움)
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import time
from typing import Optional

import websockets


def effective_ros_main_id_for_json(cli_ros_main_id: Optional[int]) -> Optional[int]:
    """CLI 우선, 없으면 ROS_DOMAIN_ID 환경변수, 둘 다 없으면 None (JSON 키 생략)."""
    if cli_ros_main_id is not None:
        return int(cli_ros_main_id)
    raw = os.environ.get("ROS_DOMAIN_ID", "").strip()
    if raw == "":
        return None
    try:
        return int(raw)
    except ValueError:
        return None


def _inject_ros_main_id(payload: dict, ros_main_id: Optional[int]) -> None:
    if ros_main_id is not None:
        payload["ros_main_id"] = ros_main_id


def _print_connection_refused_help(uri: str) -> None:
    import sys

    print(
        "연결 거부(Connection refused): 이 주소에서 웹소켓 서버가 안 떠 있습니다.\n"
        f"  URI: {uri}\n"
        "  → 같은 기기에서 먼저 다른 터미널로 서버 실행:\n"
        "      source /opt/ros/<distro>/setup.bash && source ~/GPSTopicTest/install/setup.bash\n"
        "      export ROS_DOMAIN_ID=23\n"
        "      ros2 run gps_field_ws_bridge ws_gps_publisher\n"
        "  → 서버가 다른 PC/보드에 있으면: --uri ws://그쪽_IP:8765",
        file=sys.stderr,
    )


def _print_subscriber_topic_hint(payload: dict) -> None:
    rid = payload.get("ros_main_id")
    if rid is None:
        return
    try:
        n = int(rid)
    except (TypeError, ValueError):
        return
    print(
        "힌트: 이 메시지는 토픽 /pinky_{}/gps_pos 로 publish 됩니다. "
        "LCD/구독 노드도 그 토픽을 구독해야 합니다. "
        "topic_name 을 비운 기본 구독이면: export ROS_DOMAIN_ID={} "
        "또는 -p topic_name:=/pinky_{}/gps_pos".format(n, n, n),
        flush=True,
    )


async def send_once(uri: str, payload: dict) -> None:
    async with websockets.connect(uri) as ws:
        await ws.send(json.dumps(payload))
        print("sent:", payload)
        _print_subscriber_topic_hint(payload)


async def send_loop(
    uri: str,
    rate_hz: float,
    frame_id: str,
    pinky_id: Optional[int],
    ros_main_id: Optional[int],
) -> None:
    period = 1.0 / max(rate_hz, 0.1)
    t0 = time.monotonic()
    n = 0
    async with websockets.connect(uri) as ws:
        while True:
            t = time.monotonic() - t0
            x = 940.0 + 800.0 * math.sin(t * 0.5)
            y = 705.0 + 600.0 * math.cos(t * 0.4)
            payload: dict = {
                "x_mm": x,
                "y_mm": y,
                "yaw_deg": math.degrees(math.sin(t * 0.3)) * 30.0,
                "frame_id": frame_id,
            }
            if pinky_id is not None:
                payload["pinky_id"] = pinky_id
            _inject_ros_main_id(payload, ros_main_id)
            await ws.send(json.dumps(payload))
            n += 1
            if n % int(max(rate_hz, 1)) == 0:
                print(f"tick {n} x={x:.1f} y={y:.1f}")
            await asyncio.sleep(period)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--uri", default="ws://127.0.0.1:8765", help="WebSocket URI")
    p.add_argument(
        "--pinky-id",
        type=int,
        default=None,
        help="Optional; if omitted, server fills msg.pinky_id from ROS_DOMAIN_ID",
    )
    p.add_argument("--x", type=float, default=500.0)
    p.add_argument("--y", type=float, default=400.0)
    p.add_argument("--yaw", type=float, default=0.0)
    p.add_argument(
        "--frame-id", default="field_1880x1410", help="optional; server has default"
    )
    p.add_argument(
        "--ros-main-id",
        type=int,
        default=None,
        metavar="N",
        help="JSON ros_main_id(토픽 /pinky_N/...). 생략 시 환경변수 ROS_DOMAIN_ID 사용, 그것도 없으면 키 생략",
    )
    p.add_argument(
        "--loop",
        action="store_true",
        help="continuous sine path at ~5 Hz (ignores --x/--y unless not loop)",
    )
    p.add_argument("--rate", type=float, default=5.0, help="with --loop, Hz")
    p.epilog = (
        "한 번만 전송: --loop 를 붙이지 않음 (기본). "
        "연속 전송: --loop [--rate 5]."
    )
    args = p.parse_args()
    ros_main_id = effective_ros_main_id_for_json(args.ros_main_id)

    try:
        if args.loop:
            asyncio.run(
                send_loop(
                    args.uri,
                    args.rate,
                    args.frame_id,
                    args.pinky_id,
                    ros_main_id,
                )
            )
        else:
            payload = {
                "x_mm": args.x,
                "y_mm": args.y,
                "yaw_deg": args.yaw,
                "frame_id": args.frame_id,
            }
            if args.pinky_id is not None:
                payload["pinky_id"] = args.pinky_id
            _inject_ros_main_id(payload, ros_main_id)
            asyncio.run(send_once(args.uri, payload))
    except ConnectionRefusedError:
        _print_connection_refused_help(args.uri)
        raise SystemExit(1) from None


if __name__ == "__main__":
    main()
