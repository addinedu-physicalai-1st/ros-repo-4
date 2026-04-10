from __future__ import annotations

from collections import deque
import json
import math
import time
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from ...config import AppConfig
from ...vision.aruco_utils import (
    apply_homography,
    detect_aruco_markers_scaled,
    draw_aruco_detections,
    solve_homography_from_4_markers_custom,
)
from ...ws_sender import RobotWsSender
from .camera_view import CameraView


class FieldTab(QWidget):
    def rebuild_pinky_combo(self) -> None:
        self._rebuild_pinky_combo()

    def __init__(self, cfg: AppConfig) -> None:
        super().__init__()
        self._cfg = cfg
        self._H: Optional[np.ndarray] = None
        self._last_robots: list[dict[str, Any]] = []
        self._abs_heading_click0_mm: Optional[tuple[float, float]] = None
        self._last_overlay_bgr: Optional[np.ndarray] = None
        self._frame_idx: int = 0
        # Rolling detection history per pinky id (for confidence %)
        self._pinky_hist_len: int = 30
        self._pinky_seen_hist: dict[int, deque[int]] = {}

        self._view = CameraView()
        self._view.set_overlay_callback(self._overlay)
        self._view.set_click_callback(self._on_image_click)

        self._btn_start = QPushButton("카메라 시작")
        self._btn_stop = QPushButton("카메라 정지")
        self._btn_ws_start = QPushButton("WS 스레드 시작")
        self._btn_ws_stop = QPushButton("WS 스레드 정지")
        self._btn_snap = QPushButton("스냅샷 저장")
        self._btn_reset_abs = QPushButton("절대 회전 기준 리셋")

        self._chk_map_only = QCheckBox("맵 등록 마커만 표시/처리")
        self._chk_pinky_only = QCheckBox("핑키 ID만 표시/처리")
        self._chk_rect = QCheckBox("지도영역 Rect(빨강, 디버그)")
        self._chk_abs_line = QCheckBox("절대 기준선(녹색) 표시")
        self._chk_flip_lr = QCheckBox("좌우 반전")
        self._chk_flip_ud = QCheckBox("상하 반전")
        self._chk_pos = QCheckBox("Pos (x,y,yaw 표시)")
        self._chk_ws = QCheckBox("위치 전송 (WebSocket)")
        self._chk_xy_calib = QCheckBox("보정: 지도 클릭으로 핑키 x,y 오프셋(mm) 추가")
        self._chk_abs_heading = QCheckBox("절대 회전 기준(2점 클릭) 설정")

        self._chk_map_only.setChecked(cfg.ui_filter_map_only)
        self._chk_pinky_only.setChecked(cfg.ui_filter_pinky_only)
        self._chk_rect.setChecked(cfg.ui_debug_map_rect)
        self._chk_abs_line.setChecked(cfg.ui_show_abs_heading_line)
        self._chk_flip_lr.setChecked(cfg.camera_flip_lr)
        self._chk_flip_ud.setChecked(cfg.camera_flip_ud)
        self._chk_pos.setChecked(cfg.ui_show_pos)
        self._chk_ws.setChecked(cfg.ui_ws_send)

        self._view.set_flip(lr=cfg.camera_flip_lr, ud=cfg.camera_flip_ud)

        self._combo_pinky = QComboBox()
        self._rebuild_pinky_combo()

        self._lbl_status = QLabel("호모그래피: 대기")
        self._lbl_status.setWordWrap(True)
        # 정보량이 바뀌어도 레이아웃이 흔들리지 않도록 고정 높이 멀티라인 영역 사용
        self._info_box = QPlainTextEdit()
        self._info_box.setReadOnly(True)
        self._info_box.setMinimumHeight(150)
        self._info_box.setMaximumHeight(150)
        self._info_box.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        self._lbl_abs = QLabel(f"절대 회전 기준(deg) = {self._cfg.abs_heading_deg:.2f}")
        self._lbl_abs.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)

        self._sender = RobotWsSender()
        self._ws_thread_on = False

        self._build()
        self._wire()

    def _rebuild_pinky_combo(self) -> None:
        self._combo_pinky.clear()
        for pid in self._cfg.pinky_marker_ids:
            name = self._cfg.pinky_names.get(pid, "")
            self._combo_pinky.addItem(f"{pid} ({name})" if name else str(pid), pid)

    def _build(self) -> None:
        root = QVBoxLayout(self)
        root.addWidget(self._view, stretch=1)

        g = QGroupBox("필터 / 전송 / 보정")
        gl = QGridLayout(g)
        r = 0
        gl.addWidget(self._chk_map_only, r, 0)
        gl.addWidget(self._chk_pinky_only, r, 1)
        r += 1
        gl.addWidget(self._chk_rect, r, 0)
        gl.addWidget(self._chk_abs_line, r, 1)
        r += 1
        gl.addWidget(self._chk_flip_lr, r, 0)
        gl.addWidget(self._chk_flip_ud, r, 1)
        r += 1
        gl.addWidget(self._chk_pos, r, 0)
        gl.addWidget(self._chk_ws, r, 1)
        r += 1
        gl.addWidget(self._chk_xy_calib, r, 1)
        r += 1
        gl.addWidget(self._chk_abs_heading, r, 0)
        gl.addWidget(self._btn_reset_abs, r, 1)
        r += 1
        gl.addWidget(QLabel("보정 대상 핑키"), r, 0)
        gl.addWidget(self._combo_pinky, r, 1)
        root.addWidget(g)

        row = QHBoxLayout()
        row.addWidget(self._btn_start)
        row.addWidget(self._btn_stop)
        row.addWidget(self._btn_ws_start)
        row.addWidget(self._btn_ws_stop)
        row.addWidget(self._btn_snap)
        root.addLayout(row)

        root.addWidget(self._lbl_status)
        root.addWidget(self._lbl_abs)
        root.addWidget(self._info_box)

    def _wire(self) -> None:
        self._btn_start.clicked.connect(self._on_start)
        self._btn_stop.clicked.connect(self._on_stop)
        self._btn_ws_start.clicked.connect(self._on_ws_start)
        self._btn_ws_stop.clicked.connect(self._on_ws_stop)
        self._btn_snap.clicked.connect(self._on_snapshot)
        self._btn_reset_abs.clicked.connect(self._on_reset_abs_heading)
        self._chk_map_only.toggled.connect(lambda v: setattr(self._cfg, "ui_filter_map_only", v))
        self._chk_pinky_only.toggled.connect(lambda v: setattr(self._cfg, "ui_filter_pinky_only", v))
        self._chk_rect.toggled.connect(lambda v: setattr(self._cfg, "ui_debug_map_rect", v))
        self._chk_abs_line.toggled.connect(lambda v: setattr(self._cfg, "ui_show_abs_heading_line", v))
        self._chk_pos.toggled.connect(lambda v: setattr(self._cfg, "ui_show_pos", v))
        self._chk_ws.toggled.connect(lambda v: setattr(self._cfg, "ui_ws_send", v))
        self._chk_abs_heading.toggled.connect(self._on_toggle_abs_heading)
        self._chk_flip_lr.toggled.connect(self._on_flip_changed)
        self._chk_flip_ud.toggled.connect(self._on_flip_changed)

    def _on_start(self) -> None:
        self._rebuild_pinky_combo()
        self._cfg.camera_flip_lr = bool(self._chk_flip_lr.isChecked())
        self._cfg.camera_flip_ud = bool(self._chk_flip_ud.isChecked())
        self._view.set_flip(lr=self._cfg.camera_flip_lr, ud=self._cfg.camera_flip_ud)
        ok, msg = self._view.start(
            self._cfg.camera_index,
            self._cfg.camera_width,
            self._cfg.camera_height,
            target_fps=self._cfg.camera_target_fps,
        )
        self._lbl_status.setText(f"{msg} | 설정 탭 해상도·인덱스 확인")

    def _on_stop(self) -> None:
        self._view.stop()
        self._lbl_status.setText("카메라 정지")

    def _on_flip_changed(self, _checked: bool) -> None:
        self._cfg.camera_flip_lr = bool(self._chk_flip_lr.isChecked())
        self._cfg.camera_flip_ud = bool(self._chk_flip_ud.isChecked())
        self._view.set_flip(lr=self._cfg.camera_flip_lr, ud=self._cfg.camera_flip_ud)

    def _on_ws_start(self) -> None:
        self._sender.start()
        self._ws_thread_on = True

    def _on_ws_stop(self) -> None:
        self._sender.stop()
        self._ws_thread_on = False

    def _on_toggle_abs_heading(self, on: bool) -> None:
        if on:
            self._abs_heading_click0_mm = None
            self._lbl_status.setText("절대 회전 기준 설정: 지도에서 2점을 클릭하세요(+X 방향).")

    def _on_reset_abs_heading(self) -> None:
        self._cfg.abs_heading_deg = 0.0
        self._abs_heading_click0_mm = None
        self._cfg.abs_heading_p0_mm = None
        self._cfg.abs_heading_p1_mm = None
        self._lbl_abs.setText(f"절대 회전 기준(deg) = {self._cfg.abs_heading_deg:.2f}")

    def _on_snapshot(self) -> None:
        # overlay가 포함된 프레임을 저장 (현재 보이는 화면 기준)
        img = self._last_overlay_bgr
        if img is None:
            return
        out_dir = Path.cwd() / "snapshots"
        out_dir.mkdir(parents=True, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        png_path = out_dir / f"snapshot_{ts}.png"
        json_path = out_dir / f"snapshot_{ts}.json"
        cv2.imwrite(str(png_path), img)
        meta = {
            "stamp_ms": int(time.time() * 1000),
            "abs_heading_deg": float(self._cfg.abs_heading_deg),
            "robots": self._last_robots,
            "map_marker_ids": list(self._cfg.map_marker_ids),
            "map_corner_mm": [[float(x), float(y)] for x, y in self._cfg.map_corner_mm],
        }
        if self._H is not None:
            meta["H_px_to_mm"] = self._H.astype(float).tolist()
        json_path.write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
        self._lbl_status.setText(f"스냅샷 저장: {png_path.name}")

    def _on_image_click(self, ix: float, iy: float) -> None:
        H = self._H
        if H is None:
            return

        # 절대 회전 기준 설정(2점)
        if self._chk_abs_heading.isChecked():
            p_mm = apply_homography(H, (ix, iy))
            if any(math.isnan(v) for v in p_mm):
                return
            if self._abs_heading_click0_mm is None:
                self._abs_heading_click0_mm = (float(p_mm[0]), float(p_mm[1]))
                self._lbl_status.setText("절대 회전 기준: 첫 점 OK. 두 번째 점을 클릭하세요.")
                return
            p0 = self._abs_heading_click0_mm
            p1 = (float(p_mm[0]), float(p_mm[1]))
            dx = p1[0] - p0[0]
            dy = p1[1] - p0[1]
            if abs(dx) + abs(dy) < 1e-6:
                return
            heading = float(math.degrees(math.atan2(dy, dx)))
            self._cfg.abs_heading_deg = heading
            self._cfg.abs_heading_p0_mm = (float(p0[0]), float(p0[1]))
            self._cfg.abs_heading_p1_mm = (float(p1[0]), float(p1[1]))
            self._abs_heading_click0_mm = None
            self._lbl_abs.setText(f"절대 회전 기준(deg) = {self._cfg.abs_heading_deg:.2f}")
            self._lbl_status.setText("절대 회전 기준 설정 완료.")
            return

        # 핑키 위치(x,y) 오프셋 보정
        if not self._chk_xy_calib.isChecked():
            return
        raw_pid = self._combo_pinky.currentData()
        if raw_pid is None:
            return
        pid = int(raw_pid)
        # 클릭 지점 mm / 현재 마커 중심 mm 차이를 오프셋에 누적
        dets = getattr(self, "_last_dets_full", [])
        det = next((d for d in dets if d.marker_id == pid), None)
        if det is None:
            return
        click_mm = apply_homography(H, (ix, iy))
        cur_mm = apply_homography(H, det.center_px)
        if any(math.isnan(v) for v in (*click_mm, *cur_mm)):
            return
        ddx = float(click_mm[0] - cur_mm[0])
        ddy = float(click_mm[1] - cur_mm[1])
        old = self._cfg.pinky_offsets_mm.get(pid, (0.0, 0.0))
        self._cfg.pinky_offsets_mm[pid] = (old[0] + ddx, old[1] + ddy)

    def _filter_draw_ids(
        self,
        dets,
        mset: set[int],
        pset: set[int],
    ):
        fm = self._cfg.ui_filter_map_only
        fp = self._cfg.ui_filter_pinky_only
        out = []
        for d in dets:
            mid = d.marker_id
            if fm and fp:
                if mid in mset or mid in pset:
                    out.append(d)
            elif fm:
                if mid in mset:
                    out.append(d)
            elif fp:
                if mid in pset:
                    out.append(d)
            else:
                out.append(d)
        return out

    def _overlay(self, bgr: np.ndarray) -> None:
        cfg = self._cfg
        self._frame_idx += 1
        dets = detect_aruco_markers_scaled(bgr, cfg.aruco_dict, max_w=1280)
        self._last_dets_full = dets

        ids_map = list(cfg.map_marker_ids)[:4]
        corners_mm = list(cfg.map_corner_mm)[:4]
        if len(ids_map) != 4 or len(corners_mm) != 4:
            self._H = None
            self._lbl_status.setText("맵 마커 ID / 코너 설정이 4개 필요합니다.")
            return

        H = solve_homography_from_4_markers_custom(dets, ids_map, corners_mm)
        self._H = H
        if H is None:
            self._lbl_status.setText("호모그래피 실패 — 맵 마커 4개가 화면에 필요합니다.")
        else:
            self._lbl_status.setText("호모그래피 OK")

        mset = set(cfg.map_marker_ids)
        pset = set(cfg.pinky_marker_ids)
        to_draw = self._filter_draw_ids(dets, mset, pset)
        draw_aruco_detections(bgr, to_draw)

        if H is not None and cfg.ui_debug_map_rect:
            # 중심이 아니라 각 맵 마커의 4코너를 모두 포함 → 마커 전체가 빨간 영역 안에 들어감
            pts: list[list[int]] = []
            need = set(ids_map)
            for d in dets:
                if d.marker_id in need:
                    for corner in d.corners_px:
                        pts.append([int(round(corner[0])), int(round(corner[1]))])
            if len(pts) >= 2:
                arr = np.array(pts, dtype=np.int32)
                x, y, w, h = cv2.boundingRect(arr)
                pad = max(0, int(cfg.map_debug_rect_padding_px))
                Himg, Wimg = bgr.shape[:2]
                x0 = max(0, x - pad)
                y0 = max(0, y - pad)
                x1 = min(Wimg - 1, x + w + pad)
                y1 = min(Himg - 1, y + h + pad)
                cv2.rectangle(bgr, (x0, y0), (x1, y1), (0, 0, 255), 3)

        if H is not None and cfg.ui_show_abs_heading_line and cfg.abs_heading_p0_mm and cfg.abs_heading_p1_mm:
            try:
                Hinv = np.linalg.inv(H)
                p0 = np.array([cfg.abs_heading_p0_mm[0], cfg.abs_heading_p0_mm[1], 1.0], dtype=np.float64).reshape(
                    (3, 1)
                )
                p1 = np.array([cfg.abs_heading_p1_mm[0], cfg.abs_heading_p1_mm[1], 1.0], dtype=np.float64).reshape(
                    (3, 1)
                )
                q0 = Hinv @ p0
                q1 = Hinv @ p1
                if abs(float(q0[2, 0])) > 1e-9 and abs(float(q1[2, 0])) > 1e-9:
                    u0 = int(round(float(q0[0, 0] / q0[2, 0])))
                    v0 = int(round(float(q0[1, 0] / q0[2, 0])))
                    u1 = int(round(float(q1[0, 0] / q1[2, 0])))
                    v1 = int(round(float(q1[1, 0] / q1[2, 0])))
                    # 가시성: 흰 외곽선 + 파란 선(두껍게)
                    cv2.line(bgr, (u0, v0), (u1, v1), (255, 255, 255), 14)
                    cv2.line(bgr, (u0, v0), (u1, v1), (255, 0, 0), 10)
                    cv2.circle(bgr, (u0, v0), 12, (255, 255, 255), -1)
                    cv2.circle(bgr, (u0, v0), 9, (255, 0, 0), -1)
                    cv2.circle(bgr, (u1, v1), 12, (255, 255, 255), -1)
                    cv2.circle(bgr, (u1, v1), 9, (255, 0, 0), -1)
            except Exception:
                pass

        if H is not None:
            got_map = {d.marker_id: d for d in dets if d.marker_id in set(ids_map)}
            for mid in ids_map:
                if mid not in got_map:
                    continue
                d = got_map[mid]
                u, v = d.center_px
                xm, ym = apply_homography(H, d.center_px)
                label = f"({mid}: {u:.0f},{v:.0f}px | {xm:.0f},{ym:.0f}mm)"
                cv2.putText(
                    bgr,
                    label,
                    (int(u) + 8, int(v) + 22),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

        robots: list[dict[str, Any]] = []
        # Confidence / validity should be computed even if H is None
        detections_by_id = {d.marker_id: d for d in dets}
        for pid in cfg.pinky_marker_ids:
            det = detections_by_id.get(pid)
            seen = 1 if det is not None else 0
            hist = self._pinky_seen_hist.get(pid)
            if hist is None:
                hist = deque(maxlen=self._pinky_hist_len)
                self._pinky_seen_hist[pid] = hist
            hist.append(seen)
            conf = int(round(100.0 * (sum(hist) / max(1, len(hist)))))

            robots.append(
                {
                    "pinky_id": int(pid),
                    "name": cfg.pinky_names.get(pid, ""),
                    "x_mm": 0.0,
                    "y_mm": 0.0,
                    "yaw_deg": 0.0,
                    "yaw_abs_deg": 0.0,
                    "confidence": conf if seen else 0,
                    "is_valid": bool(seen),
                }
            )

        if H is not None:
            # 맵 영역(4코너) 안에 들어오는 핑키만 Pos/전송 대상으로 포함
            poly_mm = [(float(x), float(y)) for x, y in cfg.map_corner_mm[:4]]

            def _in_convex_quad(pt: tuple[float, float], quad: list[tuple[float, float]]) -> bool:
                # 4점이 볼록/오목이어도 동작하는 간단한 ray casting
                x, y = float(pt[0]), float(pt[1])
                inside = False
                n = len(quad)
                for i in range(n):
                    x1, y1 = quad[i]
                    x2, y2 = quad[(i + 1) % n]
                    # y가 변을 가로지르는지 검사
                    if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / ((y2 - y1) + 1e-12) + x1):
                        inside = not inside
                return inside

            # Fill pose for detected pinkies (others stay is_valid=False and pose=0)
            robots_by_id = {int(r["pinky_id"]): r for r in robots}
            for pid in cfg.pinky_marker_ids:
                det = detections_by_id.get(pid)
                if det is None:
                    continue
                x_mm, y_mm = apply_homography(H, det.center_px)
                off = cfg.pinky_offsets_mm.get(pid, (0.0, 0.0))
                x_mm += off[0]
                y_mm += off[1]
                # yaw: 이미지가 아니라 맵(mm) 좌표계에서 계산 후, 절대 회전 기준 대비 상대각으로 변환
                c0_mm = apply_homography(H, (det.corners_px[0][0], det.corners_px[0][1]))
                c1_mm = apply_homography(H, (det.corners_px[1][0], det.corners_px[1][1]))
                dx = float(c1_mm[0] - c0_mm[0])
                dy = float(c1_mm[1] - c0_mm[1])
                yaw_abs = float(math.degrees(math.atan2(dy, dx)))
                yaw_rel = float(((yaw_abs - float(cfg.abs_heading_deg) + 180.0) % 360.0) - 180.0)
                r = robots_by_id.get(int(pid))
                if r is None:
                    continue
                # 맵 밖이면 좌표는 0 유지하되, is_valid/confidence는 유지 (마커 감지는 됨)
                if _in_convex_quad((float(x_mm), float(y_mm)), poly_mm):
                    r["x_mm"] = float(x_mm)
                    r["y_mm"] = float(y_mm)
                    # 전송/표시는 상대각(yaw_rel)로 사용
                    r["yaw_deg"] = float(yaw_rel)
                    r["yaw_abs_deg"] = float(yaw_abs)

        self._last_robots = robots
        self._last_overlay_bgr = bgr.copy()

        # WebSocket: N프레임마다 전송 (에디터 Pos는 매 프레임 갱신됨)
        n_ws = max(1, int(cfg.ws_send_every_n_frames))
        if cfg.ui_ws_send and self._ws_thread_on and (self._frame_idx % n_ws == 0):
            self._sender.update_robots(cfg.ws_uri, cfg.frame_id, robots)

        if cfg.ui_show_pos:
            lines = [
                f"[frame {self._frame_idx} | 목표 {cfg.camera_target_fps}fps | "
                f"WS {n_ws}프레임마다]",
                f"[Pos] abs_heading={cfg.abs_heading_deg:.2f}°",
            ]
            for r in robots:
                lines.append(
                    f"  id={r['pinky_id']} {r['name']}: "
                    f"x={r['x_mm']:.1f} y={r['y_mm']:.1f} yaw={r['yaw_deg']:.1f}° "
                    f"valid={int(bool(r.get('is_valid')))} conf={int(r.get('confidence', 0))}"
                )
            self._info_box.setPlainText("\n".join(lines))
        else:
            self._info_box.setPlainText("")