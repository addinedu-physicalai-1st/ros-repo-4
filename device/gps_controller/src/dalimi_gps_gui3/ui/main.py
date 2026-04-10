from __future__ import annotations

import sys
from pathlib import Path

from PyQt6.QtWidgets import QApplication

from ..config import load_config, save_config
from .window import MainWindow


def run() -> None:
    app = QApplication(sys.argv)

    root = Path.cwd()
    env_path = root / ".env"
    if not env_path.exists():
        example = root / ".env.example"
        if example.exists():
            env_path.write_text(example.read_text(encoding="utf-8"), encoding="utf-8")
        else:
            env_path.write_text("", encoding="utf-8")

    cfg = load_config(env_path)
    w = MainWindow(cfg)
    w.show()

    code = app.exec()
    save_config(w.config_snapshot())
    raise SystemExit(code)
