#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""WebSocket server: JSON → PinkyGps on /pinky_{domain}/gps_pos.

- DDS 격리: 모든 노드의 ``ROS_DOMAIN_ID`` 가 같아야 서로 메시지를 봅니다.
- 토픽 문자열의 숫자(예: ``/pinky_23/gps_pos``)는 ROS가 미리 등록할 필요가 없고,
  퍼블리셔·구독자가 **같은 이름**을 쓰면 됩니다. “없는 토픽 id”처럼 보이는 경우는
  대개 **구독 쪽 토픽 이름과 불일치**입니다.

기본 동작: JSON 에 ``ros_main_id`` 가 있으면 그 정수로 **토픽** ``/pinky_{id}/gps_pos`` 를 고릅니다.
파라미터 ``topic_domain_from_env_only:=true`` 이면 토픽 숫자는 **항상 서버의**
``ROS_DOMAIN_ID`` 만 쓰고, ``ros_main_id`` 는 ``pinky_id`` 등 메시지 내용 정렬용으로만 씁니다.

``topic_name`` 이 비어 있지 않으면 기본적으로 **그 토픽 하나**만 쓰며 JSON ``ros_main_id`` 는 무시합니다.
다만 ``pinky_ids_env_file`` 로 읽은 **id 목록이 비어 있지 않으면** 그것이 우선하고,
이 경우 ``topic_name`` 은 무시됩니다 (런치에 ``topic_name`` 이 박혀 있어도 .env 다중 발행 가능).

파라미터 ``pinky_ids_env_file`` 로 ``.env`` 를 지정하고, 그 안의 키(기본
``PINKY_GPS_PUBLISH_DOMAIN_IDS``)에 ``1,2,3`` 형식이면 **한 번 수신한 포즈를**
``topic_pattern`` 기준으로 위 id 들마다 publish 합니다.

기본은 목록 **전체**에 동일 메시지를 냅니다. ``domain_ids_env_filter_by_ros_main_id:=true`` 이면
JSON 에 ``ros_main_id`` 가 있을 때 **그 id 가 목록에 포함된 경우에만** 해당 토픽 **한 곳**에만
내고, ``ros_main_id`` 가 없으면 기존처럼 목록 전체에 브로드캐스트합니다.
"""

from __future__ import annotations

import asyncio
import errno
import json
import os
import queue
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from gps_field_msgs.msg import PinkyGps

from gps_field_ws_bridge.ros_domain_topic import resolve_gps_topic


def _load_env_file(path: str) -> Dict[str, str]:
    """KEY=VALUE 형식만 처리 (# 주석, 따옴표 제거). python-dotenv 미사용."""
    out: Dict[str, str] = {}
    p = Path(path)
    if not p.is_file():
        return out
    text = p.read_text(encoding="utf-8", errors="replace")
    for line in text.splitlines():
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        if "=" not in s:
            continue
        key, _, val = s.partition("=")
        key = key.strip()
        val = val.strip()
        if len(val) >= 2 and val[0] == val[-1] and val[0] in "\"'":
            val = val[1:-1]
        out[key] = val
    return out


def _parse_domain_id_list(raw: str) -> List[int]:
    """'1,2,3' 또는 '1; 2 ; 3' → 정수 리스트 (잘못된 토큰은 건너뜀)."""
    ids: List[int] = []
    for p in raw.replace(";", ",").split(","):
        p = p.strip()
        if not p:
            continue
        try:
            ids.append(int(p))
        except ValueError:
            continue
    return ids


def _resolve_env_file_path(configured: str) -> str:
    p = Path(configured.strip())
    if p.is_absolute():
        return str(p)
    return str(Path.cwd() / p)


def _pinky_ids_env_candidates(env_file_param: str) -> List[str]:
    """우선 사용자 경로, `.env` 요청이면 패키지 share 의 config/.env.example 도 시도."""
    primary = _resolve_env_file_path(env_file_param)
    out: List[str] = [primary]
    base = Path(env_file_param.strip()).name
    if base == ".env" or env_file_param.strip() == ".env":
        try:
            from ament_index_python.packages import get_package_share_directory

            alt = (
                Path(get_package_share_directory("gps_field_ws_bridge"))
                / "config"
                / ".env.example"
            )
            s = str(alt)
            if s not in out:
                out.append(s)
        except Exception:
            pass
    return out


def _load_multi_publish_pinky_ids(
    env_file_param: str,
    env_key: str,
    logger,
) -> tuple[List[int], Optional[str]]:
    """(domain id 리스트, 실제로 읽은 파일 경로 또는 None)."""
    if not env_file_param.strip():
        return [], None
    candidates = _pinky_ids_env_candidates(env_file_param)
    chosen: Optional[str] = None
    for c in candidates:
        if Path(c).is_file():
            chosen = c
            break
    if chosen is None:
        logger.warning(
            "pinky_ids_env_file 을 찾을 수 없음. 시도 경로: "
            f"{candidates!r} — 작업 디렉터리에 .env 를 두거나 "
            "`cp $(ros2 pkg prefix gps_field_ws_bridge)/share/gps_field_ws_bridge/config/.env.example .env`"
        )
        return [], None
    if chosen != candidates[0]:
        logger.info(
            f"pinky_ids_env_file 폴백: {chosen!r} 사용 "
            f"(요청한 경로 {candidates[0]!r} 없음)"
        )
    env_map = _load_env_file(chosen)
    # 키 이름 변경 호환:
    # - 신규: PINKY_GPS_PUBLISH_IDS
    # - 구버전: PINKY_GPS_PUBLISH_DOMAIN_IDS
    raw_ids = env_map.get(env_key, "")
    if (not raw_ids) and env_key == "PINKY_GPS_PUBLISH_IDS":
        legacy = env_map.get("PINKY_GPS_PUBLISH_DOMAIN_IDS", "")
        if legacy:
            logger.warning(
                "레거시 키 'PINKY_GPS_PUBLISH_DOMAIN_IDS'를 발견했습니다. "
                "새 키 'PINKY_GPS_PUBLISH_IDS'로 바꿔주세요."
            )
            raw_ids = legacy
    ids = _parse_domain_id_list(raw_ids)
    if not ids:
        logger.warning(
            f"{chosen!r} 에서 키 {env_key!r} 값이 비었거나 유효한 정수 id 가 없음: {raw_ids!r}"
        )
    return ids, chosen


def _topic_names_from_pattern(topic_pattern: str, ids: List[int]) -> List[str]:
    out: List[str] = []
    for i in ids:
        t = topic_pattern.format(pinky_id=int(i), domain_id=int(i))
        if not t.startswith("/"):
            t = "/" + t
        out.append(t)
    return out


def _print_env_ids_config_first(
    logger,
    env_file_param: str,
    env_key: str,
    loaded_path: Optional[str],
    pinky_ids: List[int],
    filter_by_ros_main_id: bool,
    topic_pattern: str,
) -> None:
    """터미널에 먼저 찍어 적용 여부를 바로 확인할 수 있게 함."""
    sep = "=" * 62
    print(sep, flush=True)
    print("ws_gps_publisher — [.env pinky_ids] 적용 값 (가장 먼저 확인)", flush=True)
    print(sep, flush=True)
    if not env_file_param.strip():
        print("  pinky_ids_env_file: (비어 있음) → .env 다중 토픽 미사용", flush=True)
        print(sep, flush=True)
        logger.info(
            "[.env pinky_ids] pinky_ids_env_file 미지정 — "
            "「.env 파일을 읽었/못했」 분기 없음"
        )
        return
    print(f"  pinky_ids_env_file 파라미터: {env_file_param!r}", flush=True)
    print(f"  domain_ids_env_key: {env_key!r}", flush=True)
    if loaded_path is None:
        print("  .env 파일을 읽지 못했습니다.", flush=True)
        print("  실제 읽은 파일: (없음 — 경고는 위 로그 참고)", flush=True)
        print("  파싱된 pinky_ids: []", flush=True)
    else:
        print("  .env 파일을 읽었습니다.", flush=True)
        print(f"  실제 읽은 파일: {loaded_path!r}", flush=True)
        raw_map = _load_env_file(loaded_path)
        raw_val = raw_map.get(env_key, "")
        print(f"  키 {env_key!r} 원문: {raw_val!r}", flush=True)
        print(f"  파싱된 pinky_ids: {pinky_ids!r}", flush=True)
    print(f"  domain_ids_env_filter_by_ros_main_id: {filter_by_ros_main_id}", flush=True)
    if pinky_ids:
        names = _topic_names_from_pattern(topic_pattern, pinky_ids)
        print(f"  예상 ROS 토픽: {', '.join(names)}", flush=True)
    print(sep, flush=True)
    if loaded_path is not None:
        logger.info(".env 파일을 읽었습니다.")
    else:
        logger.warning(".env 파일을 읽지 못했습니다.")
    logger.info(
        f"[.env pinky_ids] path={loaded_path!r} key={env_key!r} "
        f"pinky_ids={pinky_ids!r} filter_by_ros_main_id={filter_by_ros_main_id}"
    )


def _domain_byte() -> int:
    try:
        return int(os.environ.get("ROS_DOMAIN_ID", "0")) & 0xFF
    except ValueError:
        return 0


def _server_domain_int() -> int:
    try:
        return int(os.environ.get("ROS_DOMAIN_ID", "0"))
    except ValueError:
        return 0


def _parse_pose(data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    try:
        x_mm = float(data["x_mm"])
        y_mm = float(data["y_mm"])
        yaw_deg = float(data.get("yaw_deg", 0.0))
    except (KeyError, TypeError, ValueError):
        return None
    # Optional fields (backward compatible)
    confidence: Optional[int] = None
    if "confidence" in data:
        try:
            confidence = int(data["confidence"])
        except (TypeError, ValueError):
            confidence = None
    is_valid: Optional[bool] = None
    if "is_valid" in data:
        v = data["is_valid"]
        if isinstance(v, bool):
            is_valid = v
        elif isinstance(v, (int, float)):
            is_valid = bool(int(v))
        elif isinstance(v, str):
            is_valid = v.strip().lower() in ("1", "true", "t", "yes", "y", "on")
    ros_main_id: Optional[int] = None
    if "ros_main_id" in data:
        try:
            ros_main_id = int(data["ros_main_id"])
        except (TypeError, ValueError):
            ros_main_id = None
    if "pinky_id" in data:
        try:
            pinky_id = int(data["pinky_id"]) & 0xFF
        except (TypeError, ValueError):
            return None
    elif ros_main_id is not None:
        # 토픽 /pinky_{ros_main_id}/... 와 동일 숫자로 맞춤 (LCD pinky_id 필터와 일치시키기 위함)
        pinky_id = int(ros_main_id) & 0xFF
    else:
        pinky_id = _domain_byte()
    return {
        "pinky_id": pinky_id,
        "x_mm": x_mm,
        "y_mm": y_mm,
        "yaw_deg": yaw_deg,
        "frame_id": data.get("frame_id"),
        "ros_main_id": ros_main_id,
        "confidence": confidence,
        "is_valid": is_valid,
    }


class WsGpsPublisher(Node):
    """WebSocket thread → queue → timer publishes PinkyGps (토픽은 pattern 또는 JSON ros_main_id)."""

    def __init__(self) -> None:
        super().__init__("ws_gps_publisher")
        self.declare_parameter("topic_name", "")
        self.declare_parameter("topic_pattern", "/pinky_{pinky_id}/gps_pos")
        self.declare_parameter("ws_host", "0.0.0.0")
        self.declare_parameter("ws_port", 8765)
        self.declare_parameter("default_frame_id", "field_1880x1410")
        self.declare_parameter("queue_drain_period_ms", 10)
        # true: 토픽은 항상 /pinky_{서버 ROS_DOMAIN_ID}/... 만 사용 (LCD가 export만 맞추면 됨)
        # false: JSON ros_main_id 가 있으면 /pinky_{ros_main_id}/... 로 분기 (기본)
        self.declare_parameter("topic_domain_from_env_only", False)
        # 비어 있지 않으면 해당 .env 를 읽어 다중 토픽 발행 (topic_name 고정과 배타)
        self.declare_parameter("pinky_ids_env_file", "")
        # 레거시 호환 (deprecated): domain_ids_env_file
        self.declare_parameter("domain_ids_env_file", "")
        # 신규(권장): pinky_ids_env_key
        self.declare_parameter("pinky_ids_env_key", "PINKY_GPS_PUBLISH_IDS")
        # 레거시 호환 (deprecated): domain_ids_env_key
        self.declare_parameter("domain_ids_env_key", "PINKY_GPS_PUBLISH_IDS")
        # .env 다중 발행 시: true → JSON ros_main_id 가 있으면 목록에 있을 때만 그 토픽 1곳에만 발행
        # 신규(권장): pinky_ids_env_filter_by_pinky_id
        self.declare_parameter("pinky_ids_env_filter_by_pinky_id", False)
        # 레거시 호환 (deprecated): domain_ids_env_filter_by_ros_main_id
        self.declare_parameter("domain_ids_env_filter_by_ros_main_id", False)

        self._topic_pattern = self.get_parameter("topic_pattern").get_parameter_value().string_value
        env_file_param = (
            self.get_parameter("pinky_ids_env_file").get_parameter_value().string_value
        ).strip()
        if not env_file_param:
            legacy = (
                self.get_parameter("domain_ids_env_file").get_parameter_value().string_value
            ).strip()
            if legacy:
                self.get_logger().warning(
                    "파라미터 'domain_ids_env_file'은 deprecated 입니다. "
                    "'pinky_ids_env_file'로 바꿔주세요."
                )
                env_file_param = legacy
        env_key = (
            self.get_parameter("pinky_ids_env_key").get_parameter_value().string_value
        ).strip()
        if not env_key:
            env_key = (
                self.get_parameter("domain_ids_env_key").get_parameter_value().string_value
            ).strip()
        env_key = env_key or "PINKY_GPS_PUBLISH_IDS"
        self._multi_publish_pinky_ids: List[int] = []
        # 신규 이름이 우선. (pinky_id == ros_main_id 관례이므로 동작은 동일)
        self._multi_filter_by_ros_main_id = bool(
            self.get_parameter("pinky_ids_env_filter_by_pinky_id").get_parameter_value().bool_value
        )
        if not self._multi_filter_by_ros_main_id:
            self._multi_filter_by_ros_main_id = bool(
                self.get_parameter("domain_ids_env_filter_by_ros_main_id")
                .get_parameter_value()
                .bool_value
            )
        _env_loaded_from: Optional[str] = None
        if env_file_param:
            self._multi_publish_pinky_ids, _env_loaded_from = (
                _load_multi_publish_pinky_ids(
                    env_file_param, env_key, self.get_logger()
                )
            )
        _print_env_ids_config_first(
            self.get_logger(),
            env_file_param,
            env_key,
            _env_loaded_from,
            self._multi_publish_pinky_ids,
            self._multi_filter_by_ros_main_id,
            self._topic_pattern,
        )

        explicit = self.get_parameter("topic_name").get_parameter_value().string_value
        topic = resolve_gps_topic(explicit, self._topic_pattern, self.get_logger())
        self._explicit_topic_requested = bool((explicit or "").strip())

        self._ws_host = self.get_parameter("ws_host").get_parameter_value().string_value
        self._ws_port = int(
            self.get_parameter("ws_port").get_parameter_value().integer_value
        )
        self._default_frame = (
            self.get_parameter("default_frame_id").get_parameter_value().string_value
        )
        period_ms = int(
            self.get_parameter("queue_drain_period_ms").get_parameter_value().integer_value
        )
        period_s = max(period_ms, 1) / 1000.0

        self._topic_domain_env_only = (
            self.get_parameter("topic_domain_from_env_only")
            .get_parameter_value()
            .bool_value
        )

        self._use_fixed_single_topic = (
            self._explicit_topic_requested and not self._multi_publish_pinky_ids
        )
        if self._multi_publish_pinky_ids and self._explicit_topic_requested:
            self.get_logger().warning(
                "topic_name 이 설정되어 있지만 .env 의 도메인 목록이 우선합니다. "
                f"다음 topic_name 은 사용하지 않습니다: {explicit.strip()!r}"
            )

        self._q: queue.SimpleQueue = queue.SimpleQueue()
        self._ws_rx_count = 0
        self._ws_rx_last_log = time.monotonic()
        self._pub_count = 0
        self._pub_last_log = time.monotonic()
        self._publishers_by_domain: dict[int, Any] = {}
        self._fixed_pub: Optional[Any] = None
        if self._use_fixed_single_topic:
            self._fixed_pub = self.create_publisher(PinkyGps, topic, 10)
        # 비고정 모드: 첫 publish 시점에 lazy 로 퍼블리셔 생성 (시작 시 /pinky_0 만 만든다는 오해 방지)

        self.create_timer(period_s, self._drain_queue)

        self._ws_fatal_lock = threading.Lock()
        self._ws_fatal_error: Optional[str] = None
        self._ws_bind_poll_deadline = time.monotonic() + 3.0
        self._ws_bind_check_timer = self.create_timer(0.1, self._ws_bind_poll)

        self._ws_thread = threading.Thread(target=self._ws_thread_main, daemon=True)
        self._ws_thread.start()

        if self._use_fixed_single_topic:
            self.get_logger().info(
                f"WebSocket ws://{self._ws_host}:{self._ws_port} → ROS publish {topic} "
                f"(고정 topic_name; JSON ros_main_id 무시) "
                f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', 'unset')!r}"
            )
        else:
            if self._topic_domain_env_only:
                self.get_logger().info(
                    f"WebSocket ws://{self._ws_host}:{self._ws_port} → 토픽은 항상 "
                    f"{self._topic_pattern!r} 의 서버 ROS_DOMAIN_ID 만 사용 "
                    f"(topic_domain_from_env_only=true; JSON ros_main_id 는 토픽이 아님) "
                    f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', 'unset')!r}"
                )
            else:
                if self._multi_publish_pinky_ids:
                    filt = (
                        "ros_main_id 있으면 목록 내 해당 토픽만"
                        if self._multi_filter_by_ros_main_id
                        else "매 수신마다 목록 전체"
                    )
                    self.get_logger().info(
                        f"WebSocket ws://{self._ws_host}:{self._ws_port} → ROS 다중 발행 "
                        f"{self._topic_pattern!r} pinky_ids={self._multi_publish_pinky_ids!r} "
                        f"({filt})"
                    )
                else:
                    self.get_logger().info(
                        f"WebSocket ws://{self._ws_host}:{self._ws_port} → ROS pattern "
                        f"{self._topic_pattern!r} (+JSON ros_main_id 로 토픽 분기 가능) "
                        f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', 'unset')!r}"
                    )

    def _ensure_publisher(self, domain_id: int) -> Any:
        if self._fixed_pub is not None:
            return self._fixed_pub
        if domain_id not in self._publishers_by_domain:
            t = self._topic_pattern.format(pinky_id=int(domain_id), domain_id=int(domain_id))
            if not t.startswith("/"):
                t = "/" + t
            self._publishers_by_domain[domain_id] = self.create_publisher(PinkyGps, t, 10)
            self.get_logger().info(f"PinkyGps publisher 추가: {t}")
        return self._publishers_by_domain[domain_id]

    def _drain_queue(self) -> None:
        while True:
            try:
                item = self._q.get_nowait()
            except queue.Empty:
                break
            pose = _parse_pose(item) if isinstance(item, dict) else None
            if pose is None:
                self.get_logger().warning(f"drop invalid JSON payload: {item!r}")
                continue
            msg = PinkyGps()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = str(pose["frame_id"] or self._default_frame)
            msg.pinky_id = int(pose["pinky_id"])
            msg.x_mm = float(pose["x_mm"])
            msg.y_mm = float(pose["y_mm"])
            msg.yaw_deg = float(pose["yaw_deg"])
            if pose.get("confidence") is not None:
                msg.confidence = max(0, min(100, int(pose["confidence"])))
            else:
                msg.confidence = 0
            if pose.get("is_valid") is not None:
                msg.is_valid = bool(pose["is_valid"])
            else:
                msg.is_valid = True
            rid = pose.get("ros_main_id")

            self._pub_count += 1
            now = time.monotonic()
            if now - self._pub_last_log >= 1.0:
                self._pub_last_log = now
                self.get_logger().info(
                    f"publish ok: total={self._pub_count} last_pinky_id={int(msg.pinky_id)}"
                )
            if self._fixed_pub is not None:
                domain_for_topic = _server_domain_int()
                pub = self._ensure_publisher(domain_for_topic)
                pub.publish(msg)
            elif self._multi_publish_pinky_ids:
                allowed = frozenset(self._multi_publish_pinky_ids)
                topic_targets: List[int]
                if self._multi_filter_by_ros_main_id and rid is not None:
                    try:
                        rmi = int(rid)
                    except (TypeError, ValueError):
                        rmi = None
                    if rmi is not None and rmi in allowed:
                        topic_targets = [rmi]
                    elif rmi is not None:
                        self.get_logger().warning(
                            f"ros_main_id={rmi} 가 .env 허용 목록 {sorted(allowed)} 에 없어 발행 생략"
                        )
                        topic_targets = []
                    else:
                        topic_targets = list(self._multi_publish_pinky_ids)
                else:
                    topic_targets = list(self._multi_publish_pinky_ids)
                for domain_for_topic in topic_targets:
                    pub = self._ensure_publisher(domain_for_topic)
                    pub.publish(msg)
            elif self._topic_domain_env_only:
                domain_for_topic = _server_domain_int()
                pub = self._ensure_publisher(domain_for_topic)
                pub.publish(msg)
            elif rid is None:
                domain_for_topic = _server_domain_int()
                pub = self._ensure_publisher(domain_for_topic)
                pub.publish(msg)
            else:
                try:
                    domain_for_topic = int(rid)
                except (TypeError, ValueError):
                    domain_for_topic = _server_domain_int()
                pub = self._ensure_publisher(domain_for_topic)
                pub.publish(msg)

    def _ws_bind_poll(self) -> None:
        with self._ws_fatal_lock:
            err = self._ws_fatal_error
        if err is not None:
            self.destroy_timer(self._ws_bind_check_timer)
            self.get_logger().fatal(err)
            try:
                self.destroy_node()
            except Exception:
                pass
            rclpy.shutdown()
            return
        if time.monotonic() >= self._ws_bind_poll_deadline:
            self.destroy_timer(self._ws_bind_check_timer)

    def _ws_thread_main(self) -> None:
        try:
            asyncio.run(self._ws_serve())
        except OSError as e:
            if e.errno == errno.EADDRINUSE:
                detail = (
                    f"포트 {self._ws_port} 가 이미 사용 중입니다. "
                    "다른 `ws_gps_publisher` 터미널을 종료하거나 "
                    f"`--ros-args -p ws_port:=8766` 처럼 포트를 바꾸세요."
                )
            else:
                detail = str(e)
            msg = f"WebSocket 서버를 띄울 수 없음 ({self._ws_host}:{self._ws_port}): {detail}"
            with self._ws_fatal_lock:
                self._ws_fatal_error = msg
        except Exception as e:
            with self._ws_fatal_lock:
                self._ws_fatal_error = f"WebSocket 서버 스레드 오류: {e}"

    async def _handler(self, websocket: Any) -> None:
        peer = getattr(websocket, "remote_address", None)
        self.get_logger().info(f"WS client connected {peer}")
        try:
            async for raw in websocket:
                if isinstance(raw, bytes):
                    raw = raw.decode("utf-8", errors="replace")
                try:
                    data = json.loads(raw)
                except json.JSONDecodeError as e:
                    self.get_logger().warning(f"JSON decode error: {e}")
                    continue
                if not isinstance(data, dict):
                    self.get_logger().warning("JSON root must be an object")
                    continue
                self._q.put(data)
                self._ws_rx_count += 1
                now = time.monotonic()
                if now - self._ws_rx_last_log >= 1.0:
                    self._ws_rx_last_log = now
                    # 너무 길어질 수 있어 키 목록만 출력
                    self.get_logger().info(
                        f"ws rx ok: total={self._ws_rx_count} keys={sorted(list(data.keys()))}"
                    )
        finally:
            self.get_logger().info(f"WS client disconnected {peer}")

    async def _ws_serve(self) -> None:
        import websockets

        async with websockets.serve(
            self._handler,
            self._ws_host,
            self._ws_port,
            ping_interval=20,
            ping_timeout=20,
        ):
            await asyncio.Future()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WsGpsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
