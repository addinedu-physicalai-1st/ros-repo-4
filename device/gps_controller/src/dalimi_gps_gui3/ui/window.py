from __future__ import annotations

from pathlib import Path

from PyQt6.QtWidgets import QMainWindow, QTabWidget

from ..config import AppConfig
from .widgets.field_tab import FieldTab
from .widgets.settings_tab import SettingsTab


class MainWindow(QMainWindow):
    def __init__(self, cfg: AppConfig) -> None:
        super().__init__()
        self.setWindowTitle("dalimi_GPS_GUI3 — 프레임당 Pos · N프레임마다 WS (테스트)")
        self._cfg = cfg

        self._tab_settings = SettingsTab(cfg)
        self._tab_field = FieldTab(cfg)
        self._tab_settings.saved.connect(self._tab_field.rebuild_pinky_combo)

        tabs = QTabWidget()
        tabs.addTab(self._tab_field, "현장 (카메라)")
        tabs.addTab(self._tab_settings, "설정 (.env)")
        self.setCentralWidget(tabs)
        self.resize(1280, 860)

    def config_snapshot(self) -> AppConfig:
        self._tab_settings.apply_to_config()
        return self._cfg
