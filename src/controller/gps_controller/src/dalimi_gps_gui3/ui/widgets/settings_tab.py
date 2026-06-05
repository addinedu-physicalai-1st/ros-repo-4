from __future__ import annotations

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QCheckBox,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from ...config import AppConfig, save_config


class SettingsTab(QWidget):
    saved = pyqtSignal()

    def __init__(self, cfg: AppConfig) -> None:
        super().__init__()
        self._cfg = cfg

        self._cam_idx = QSpinBox()
        self._cam_idx.setRange(0, 20)
        self._cam_w = QSpinBox()
        self._cam_w.setRange(320, 8000)
        self._cam_h = QSpinBox()
        self._cam_h.setRange(240, 8000)
        self._cam_flip_lr = QCheckBox("좌우 반전")
        self._cam_flip_ud = QCheckBox("상하 반전")

        self._dict_edit = QLineEdit()

        self._corner_x = [QDoubleSpinBox() for _ in range(4)]
        self._corner_y = [QDoubleSpinBox() for _ in range(4)]
        for s in self._corner_x + self._corner_y:
            s.setRange(-1e6, 1e6)
            s.setDecimals(2)

        self._cam_height_mm = QDoubleSpinBox()
        self._cam_height_mm.setRange(0, 1e6)
        self._cam_height_mm.setDecimals(2)

        self._rect_pad_px = QSpinBox()
        self._rect_pad_px.setRange(0, 512)
        self._rect_pad_px.setToolTip("빨간 지도 Rect: 마커 코너 포함 박스 바깥 여백(px)")

        self._abs_heading_deg = QDoubleSpinBox()
        self._abs_heading_deg.setRange(-3600.0, 3600.0)
        self._abs_heading_deg.setDecimals(2)
        self._abs_heading_deg.setToolTip("절대 회전 기준선(전역). 핑키 yaw는 이 값 대비 상대각으로 계산")

        self._map_ids = QLineEdit()
        self._pinky_ids = QLineEdit()
        self._pinky_names = QLineEdit()
        self._ws_uri = QLineEdit()
        self._cam_target_fps = QSpinBox()
        self._cam_target_fps.setRange(1, 120)
        self._cam_target_fps.setToolTip("미리보기/검출 타이머: 약 1000/fps ms (예: 30 → ~33ms)")
        self._ws_every_n = QSpinBox()
        self._ws_every_n.setRange(1, 600)
        self._ws_every_n.setToolTip("처리 프레임 기준 N프레임마다 WebSocket 전송 (1=매 프레임)")
        self._frame_id = QLineEdit()

        self._btn_save = QPushButton(".env 저장")
        self._btn_reload = QPushButton("화면에 cfg 반영")

        self._pull_from_cfg()
        self._build()
        self._btn_save.clicked.connect(self._on_save)
        self._btn_reload.clicked.connect(self._pull_from_cfg)

    def _pull_from_cfg(self) -> None:
        c = self._cfg
        self._cam_idx.setValue(c.camera_index)
        self._cam_w.setValue(c.camera_width)
        self._cam_h.setValue(c.camera_height)
        self._cam_target_fps.setValue(int(c.camera_target_fps))
        self._cam_flip_lr.setChecked(bool(c.camera_flip_lr))
        self._cam_flip_ud.setChecked(bool(c.camera_flip_ud))
        self._dict_edit.setText(c.aruco_dict)
        for i in range(4):
            if i < len(c.map_corner_mm):
                self._corner_x[i].setValue(c.map_corner_mm[i][0])
                self._corner_y[i].setValue(c.map_corner_mm[i][1])
        self._cam_height_mm.setValue(c.map_center_to_camera_mm)
        self._rect_pad_px.setValue(c.map_debug_rect_padding_px)
        self._abs_heading_deg.setValue(float(c.abs_heading_deg))
        self._map_ids.setText(",".join(map(str, c.map_marker_ids)))
        self._pinky_ids.setText(",".join(map(str, c.pinky_marker_ids)))
        self._pinky_names.setText(",".join(f"{k}:{v}" for k, v in sorted(c.pinky_names.items())))
        self._ws_uri.setText(c.ws_uri)
        self._ws_every_n.setValue(int(c.ws_send_every_n_frames))
        self._frame_id.setText(c.frame_id)

    def apply_to_config(self) -> None:
        """현재 위젯 값을 cfg 객체에 반영 (저장 없이)."""
        self._push_to_cfg()

    def _push_to_cfg(self) -> None:
        c = self._cfg
        c.camera_index = self._cam_idx.value()
        c.camera_width = self._cam_w.value()
        c.camera_height = self._cam_h.value()
        c.camera_target_fps = max(1, int(self._cam_target_fps.value()))
        c.camera_flip_lr = self._cam_flip_lr.isChecked()
        c.camera_flip_ud = self._cam_flip_ud.isChecked()
        c.aruco_dict = self._dict_edit.text().strip() or c.aruco_dict
        corners = []
        for i in range(4):
            corners.append((self._corner_x[i].value(), self._corner_y[i].value()))
        c.map_corner_mm = corners
        c.map_center_to_camera_mm = self._cam_height_mm.value()
        c.map_debug_rect_padding_px = int(self._rect_pad_px.value())
        c.abs_heading_deg = float(self._abs_heading_deg.value())

        def _parse_ids(s: str, fallback: list[int]) -> list[int]:
            out: list[int] = []
            for t in s.split(","):
                t = t.strip()
                if not t:
                    continue
                try:
                    out.append(int(t))
                except ValueError:
                    continue
            return out if out else fallback

        c.map_marker_ids = _parse_ids(self._map_ids.text(), c.map_marker_ids)
        c.pinky_marker_ids = _parse_ids(self._pinky_ids.text(), c.pinky_marker_ids)

        names: dict[int, str] = {}
        for part in self._pinky_names.text().split(","):
            part = part.strip()
            if ":" in part:
                k, _, v = part.partition(":")
                try:
                    names[int(k.strip())] = v.strip()
                except ValueError:
                    pass
        c.pinky_names = names

        c.ws_uri = self._ws_uri.text().strip() or c.ws_uri
        c.ws_send_every_n_frames = max(1, int(self._ws_every_n.value()))
        c.frame_id = self._frame_id.text().strip() or c.frame_id

    def _on_save(self) -> None:
        self._push_to_cfg()
        try:
            save_config(self._cfg)
            self.saved.emit()
            QMessageBox.information(self, "저장", f"저장됨: {self._cfg.env_path}")
        except Exception as e:
            QMessageBox.critical(self, "저장 실패", str(e))

    def _build(self) -> None:
        root = QVBoxLayout(self)
        g0 = QGroupBox("카메라")
        l0 = QGridLayout(g0)
        l0.addWidget(QLabel("인덱스"), 0, 0)
        l0.addWidget(self._cam_idx, 0, 1)
        l0.addWidget(QLabel("너비"), 0, 2)
        l0.addWidget(self._cam_w, 0, 3)
        l0.addWidget(QLabel("높이"), 0, 4)
        l0.addWidget(self._cam_h, 0, 5)
        l0.addWidget(QLabel("목표 FPS"), 1, 0)
        l0.addWidget(self._cam_target_fps, 1, 1)
        l0.addWidget(self._cam_flip_lr, 1, 2)
        l0.addWidget(self._cam_flip_ud, 1, 3)
        l0.addWidget(QLabel("ARUCO_DICT"), 1, 4)
        l0.addWidget(self._dict_edit, 1, 5)
        root.addWidget(g0)

        g1 = QGroupBox("맵 코너 (mm) — 마커 id 순서 0,1,2,3")
        l1 = QGridLayout(g1)
        for i in range(4):
            l1.addWidget(QLabel(f"ID{i} X"), i, 0)
            l1.addWidget(self._corner_x[i], i, 1)
            l1.addWidget(QLabel(f"Y"), i, 2)
            l1.addWidget(self._corner_y[i], i, 3)
        root.addWidget(g1)

        g1b = QGroupBox("탑뷰 참고 / 빨간 지도 Rect")
        l1b = QGridLayout(g1b)
        l1b.addWidget(QLabel("지도 중앙→카메라 (mm)"), 0, 0)
        l1b.addWidget(self._cam_height_mm, 0, 1)
        l1b.addWidget(QLabel("Rect 여백 (px, 코너 포함 박스 확장)"), 1, 0)
        l1b.addWidget(self._rect_pad_px, 1, 1)
        l1b.addWidget(QLabel("절대 회전 기준(deg)"), 2, 0)
        l1b.addWidget(self._abs_heading_deg, 2, 1)
        root.addWidget(g1b)

        g2 = QGroupBox("마커 ID / 이름")
        l2 = QGridLayout(g2)
        l2.addWidget(QLabel("맵 마커 IDs"), 0, 0)
        l2.addWidget(self._map_ids, 0, 1)
        l2.addWidget(QLabel("핑키 IDs"), 1, 0)
        l2.addWidget(self._pinky_ids, 1, 1)
        l2.addWidget(QLabel("핑키 이름 (id:이름,)"), 2, 0)
        l2.addWidget(self._pinky_names, 2, 1)
        root.addWidget(g2)

        g3 = QGroupBox("WebSocket")
        l3 = QGridLayout(g3)
        l3.addWidget(QLabel("WS_URI"), 0, 0)
        l3.addWidget(self._ws_uri, 0, 1)
        l3.addWidget(QLabel("N프레임마다 전송"), 0, 2)
        l3.addWidget(self._ws_every_n, 0, 3)
        l3.addWidget(QLabel("frame_id"), 1, 0)
        l3.addWidget(self._frame_id, 1, 1)
        root.addWidget(g3)

        row = QHBoxLayout()
        row.addWidget(self._btn_save)
        row.addWidget(self._btn_reload)
        row.addStretch(1)
        root.addLayout(row)
        root.addStretch(1)
