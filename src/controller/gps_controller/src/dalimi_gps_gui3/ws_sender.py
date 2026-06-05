from __future__ import annotations

import asyncio
import json
import threading
from typing import Any, Optional

import websockets


class RobotWsSender:
    """
    Qt와 분리된 스레드에서 WebSocket 송신 (GUI3: 시간(Hz) 스로틀 없음).

    `field_gps_subscriber` / `ws_gps_publisher` 호환 단일 포즈 JSON:
      {"x_mm":..., "y_mm":..., "yaw_deg":..., "frame_id":..., "pinky_id":..., "ros_main_id":...}

    robots 리스트 → **로봇 1대당 1 메시지** 전송.
    호출 측에서 **N프레임마다만** `update_robots`를 부르면 그 주기로만 나간다.
    """

    def __init__(self) -> None:
        self._thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()
        self._lock = threading.Lock()
        self._latest: Optional[tuple[str, str, list[dict[str, Any]]]] = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_evt.set()
        if self._thread:
            self._thread.join(timeout=1.0)

    def update_robots(self, uri: str, frame_id: str, robots: list[dict[str, Any]]) -> None:
        with self._lock:
            self._latest = (uri, frame_id, list(robots))

    def _run(self) -> None:
        asyncio.run(self._main())

    async def _main(self) -> None:
        ws = None
        current_uri: Optional[str] = None

        while not self._stop_evt.is_set():
            with self._lock:
                job = self._latest
                self._latest = None

            if job is None:
                await asyncio.sleep(0.002)
                continue

            uri, frame_id, robots = job
            if current_uri != uri:
                current_uri = uri
                if ws is not None:
                    try:
                        await ws.close()
                    except Exception:
                        pass
                    ws = None

            if ws is None:
                try:
                    ws = await websockets.connect(uri)
                except Exception:
                    ws = None
                    await asyncio.sleep(0.5)
                    continue

            try:
                for r in robots:
                    pid = r.get("pinky_id")
                    payload = {
                        "x_mm": float(r.get("x_mm", 0.0)),
                        "y_mm": float(r.get("y_mm", 0.0)),
                        "yaw_deg": float(r.get("yaw_deg", 0.0)),
                        "frame_id": frame_id,
                        "pinky_id": int(pid) & 0xFF if pid is not None else None,
                        "ros_main_id": int(pid) if pid is not None else None,
                    }
                    # Optional new keys (bridge stays backward compatible)
                    if "confidence" in r:
                        payload["confidence"] = int(r.get("confidence") or 0)
                    if "is_valid" in r:
                        payload["is_valid"] = bool(r.get("is_valid"))
                    payload = {k: v for k, v in payload.items() if v is not None}
                    await ws.send(json.dumps(payload, ensure_ascii=False))
            except Exception:
                try:
                    await ws.close()
                except Exception:
                    pass
                ws = None
                await asyncio.sleep(0.2)
