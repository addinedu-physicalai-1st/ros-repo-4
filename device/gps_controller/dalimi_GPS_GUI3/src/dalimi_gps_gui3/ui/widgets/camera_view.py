from __future__ import annotations

import sys
from typing import Callable, Optional

import cv2
import numpy as np
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QImage, QMouseEvent, QPixmap
from PyQt6.QtWidgets import QLabel


class CameraView(QLabel):
    def __init__(self) -> None:
        super().__init__()
        self.setMinimumSize(640, 360)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setStyleSheet("background:#111; color:#ddd;")
        self.setText("카메라 미리보기")

        self._cap: cv2.VideoCapture | None = None
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._last_bgr: np.ndarray | None = None
        self._overlay_cb = None
        self._click_cb: Optional[Callable[[float, float], None]] = None
        self._req_size: tuple[int, int] | None = None
        self._actual_size: tuple[int, int] | None = None
        self._backend_name: str | None = None
        self._flip_lr: bool = False
        self._flip_ud: bool = False

    def set_flip(self, *, lr: bool, ud: bool) -> None:
        self._flip_lr = bool(lr)
        self._flip_ud = bool(ud)

    def start(self, index: int, width: int, height: int, *, target_fps: int = 30) -> tuple[bool, str]:
        self.stop()
        idx = int(index)
        req_w, req_h = int(width), int(height)
        self._req_size = (req_w, req_h)

        def read_fourcc(_cap: cv2.VideoCapture) -> str | None:
            try:
                fourcc_int = int(_cap.get(cv2.CAP_PROP_FOURCC) or 0)
                s = "".join([chr((fourcc_int >> (8 * i)) & 0xFF) for i in range(4)])
                s = s.strip()
                return s if s else None
            except Exception:
                return None

        def configure(_cap: cv2.VideoCapture, *, prefer_fourcc: str | None) -> None:
            if prefer_fourcc:
                try:
                    _cap.set(cv2.CAP_PROP_FOURCC, float(cv2.VideoWriter_fourcc(*prefer_fourcc)))
                except Exception:
                    pass
            _cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(req_w))
            _cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(req_h))
            try:
                _cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass

        wants_4k = req_w >= 3000 or req_h >= 2000
        open_attempts: list[tuple[str, object, str | None]] = []
        if sys.platform.startswith("linux"):
            open_attempts.append(("V4L2(index)", (idx, cv2.CAP_V4L2), "MJPG" if wants_4k else None))
            open_attempts.append(("V4L2(dev)", (f"/dev/video{idx}", cv2.CAP_V4L2), "MJPG" if wants_4k else None))
            open_attempts.append(("Any(index)", (idx,), "MJPG" if wants_4k else None))
        else:
            open_attempts.append(("Any(index)", (idx,), None))

        cap: cv2.VideoCapture | None = None
        selected_label = ""
        for label, args, prefer in open_attempts:
            try:
                cap_try = cv2.VideoCapture(*args)
            except Exception:
                continue
            if not cap_try.isOpened():
                try:
                    cap_try.release()
                except Exception:
                    pass
                continue

            configure(cap_try, prefer_fourcc=prefer)

            for _ in range(5):
                ok, _ = cap_try.read()
                if ok:
                    break

            act_w = int(cap_try.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
            act_h = int(cap_try.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
            fcc = read_fourcc(cap_try)

            if wants_4k:
                if fcc == "MJPG" and act_w == req_w and act_h == req_h:
                    cap = cap_try
                    selected_label = label
                    break
                if cap is None and fcc == "MJPG":
                    cap = cap_try
                    selected_label = label
                    continue
            else:
                cap = cap_try
                selected_label = label
                break

            try:
                cap_try.release()
            except Exception:
                pass

        if cap is None or not cap.isOpened():
            return False, "카메라 열기 실패"

        for _ in range(5):
            ok, _ = cap.read()
            if ok:
                break
        act_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
        act_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
        self._actual_size = (act_w, act_h) if act_w > 0 and act_h > 0 else None
        try:
            self._backend_name = str(cap.getBackendName())
        except Exception:
            self._backend_name = None

        fourcc = read_fourcc(cap)

        self._cap = cap
        fps = max(1, int(target_fps))
        interval_ms = max(1, int(round(1000.0 / float(fps))))
        self._timer.start(interval_ms)
        msg = f"카메라 시작 OK | req={req_w}x{req_h} | 타이머≈{fps}fps({interval_ms}ms)"
        if self._actual_size is not None:
            msg += f" actual={self._actual_size[0]}x{self._actual_size[1]}"
        if fourcc:
            msg += f" fourcc={fourcc}"
        if selected_label:
            msg += f" open={selected_label}"
        if self._backend_name:
            msg += f" backend={self._backend_name}"
        return True, msg

    def stop(self) -> None:
        self._timer.stop()
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
        self._cap = None
        self._last_bgr = None
        self._backend_name = None
        self._req_size = None
        self._actual_size = None

    def set_overlay_callback(self, cb) -> None:
        self._overlay_cb = cb

    def set_click_callback(self, cb: Optional[Callable[[float, float], None]]) -> None:
        self._click_cb = cb

    def last_frame_bgr(self) -> np.ndarray | None:
        return None if self._last_bgr is None else self._last_bgr.copy()

    def actual_resolution(self) -> tuple[int, int] | None:
        return None if self._actual_size is None else (int(self._actual_size[0]), int(self._actual_size[1]))

    def mousePressEvent(self, event: QMouseEvent) -> None:
        super().mousePressEvent(event)
        if self._click_cb is None or event.button() != Qt.MouseButton.LeftButton:
            return
        xy = self._widget_to_image_xy(float(event.position().x()), float(event.position().y()))
        if xy is not None:
            self._click_cb(xy[0], xy[1])

    def _widget_to_image_xy(self, lx: float, ly: float) -> Optional[tuple[float, float]]:
        # 클릭 좌표(lx,ly)는 QLabel 위젯 좌표.
        # 표시되는 pixmap은 원본 프레임을 aspect-fit으로 스케일한 결과이므로,
        # 원본 프레임 크기 기준으로 역변환해야 정확히 원본(px) 좌표가 된다.
        pm = self.pixmap()
        if pm.isNull() or self._last_bgr is None:
            return None
        ih, iw = self._last_bgr.shape[:2]  # 원본 프레임(px)
        if iw <= 0 or ih <= 0:
            return None
        lw, lh = float(self.width()), float(self.height())
        if lw <= 0 or lh <= 0:
            return None

        scale = min(lw / float(iw), lh / float(ih))
        disp_w = float(iw) * scale
        disp_h = float(ih) * scale
        ox = (lw - disp_w) * 0.5
        oy = (lh - disp_h) * 0.5
        if lx < ox or ly < oy or lx >= ox + disp_w or ly >= oy + disp_h:
            return None
        ix = (lx - ox) / scale
        iy = (ly - oy) / scale
        # 클램프(경계 클릭 안정화)
        ix = min(max(ix, 0.0), float(iw - 1))
        iy = min(max(iy, 0.0), float(ih - 1))
        return ix, iy

    def _tick(self) -> None:
        if self._cap is None:
            return
        ok, frame = self._cap.read()
        if not ok or frame is None:
            return
        # 설치 방향/드라이버에 따라 미러링되는 경우가 있어 여기서 일괄 보정
        if self._flip_lr and self._flip_ud:
            frame = cv2.flip(frame, -1)
        elif self._flip_lr:
            frame = cv2.flip(frame, 1)
        elif self._flip_ud:
            frame = cv2.flip(frame, 0)
        self._last_bgr = frame
        show = frame.copy()
        if self._overlay_cb is not None:
            try:
                self._overlay_cb(show)
            except Exception:
                pass
        self._set_bgr(show)

    def _set_bgr(self, bgr) -> None:
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        qimg = QImage(rgb.data, w, h, 3 * w, QImage.Format.Format_RGB888)
        self.setPixmap(
            QPixmap.fromImage(qimg).scaled(
                self.size(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.FastTransformation,
            )
        )
