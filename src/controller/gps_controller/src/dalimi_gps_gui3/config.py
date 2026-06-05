from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

from dotenv import dotenv_values, set_key


def _as_int(v: Optional[str], default: int) -> int:
    try:
        return int(str(v).strip())
    except Exception:
        return default


def _as_float(v: Optional[str], default: float) -> float:
    try:
        return float(str(v).strip())
    except Exception:
        return default


def _as_str(v: Optional[str], default: str) -> str:
    s = "" if v is None else str(v)
    s = s.strip()
    return s if s != "" else default


def _as_bool01(v: Optional[str], default: bool) -> bool:
    if v is None:
        return default
    s = str(v).strip().lower()
    if s in ("1", "true", "yes", "on"):
        return True
    if s in ("0", "false", "no", "off", ""):
        return False
    return default


def _as_csv_ints(v: Optional[str], default: list[int]) -> list[int]:
    if v is None:
        return default
    raw = str(v).strip()
    if raw == "":
        return default
    out: list[int] = []
    for tok in raw.split(","):
        tok = tok.strip()
        if tok == "":
            continue
        try:
            out.append(int(tok))
        except ValueError:
            continue
    return out if out else default


def _parse_pinky_names(raw: Optional[str]) -> dict[int, str]:
    out: dict[int, str] = {}
    if not raw:
        return out
    for part in str(raw).split(","):
        part = part.strip()
        if not part or ":" not in part:
            continue
        k, _, name = part.partition(":")
        try:
            out[int(k.strip())] = name.strip()
        except ValueError:
            continue
    return out


def _parse_offsets_json(raw: Optional[str]) -> dict[int, tuple[float, float]]:
    out: dict[int, tuple[float, float]] = {}
    if not raw or not str(raw).strip():
        return out
    try:
        data = json.loads(str(raw).strip())
        if not isinstance(data, dict):
            return out
        for k, v in data.items():
            try:
                kid = int(k)
            except ValueError:
                continue
            if isinstance(v, (list, tuple)) and len(v) >= 2:
                out[kid] = (float(v[0]), float(v[1]))
    except Exception:
        pass
    return out


def _offsets_to_json(m: dict[int, tuple[float, float]]) -> str:
    return json.dumps({str(k): [float(v[0]), float(v[1])] for k, v in m.items()}, ensure_ascii=False)


def _pinky_names_to_str(m: dict[int, str]) -> str:
    return ",".join(f"{k}:{v}" for k, v in sorted(m.items()))


def default_map_corners_mm() -> list[tuple[float, float]]:
    return [
        (0.0, 0.0),
        (1880.0, 0.0),
        (1880.0, 1410.0),
        (0.0, 1410.0),
    ]


@dataclass
class AppConfig:
    env_path: Path

    camera_index: int = 0
    camera_width: int = 3840
    camera_height: int = 2160
    camera_flip_lr: bool = False
    camera_flip_ud: bool = False
    # 미리보기/검출 타이머 간격 ≈ 1000/target_fps ms (카메라 실제 FPS와 맞추려면 30 등)
    camera_target_fps: int = 30

    map_corner_mm: list[tuple[float, float]] = field(default_factory=default_map_corners_mm)
    map_center_to_camera_mm: float = 1980.0
    # 지도영역 Rect(빨강): 마커 코너를 포함한 박스 바깥으로 추가 여백(px)
    map_debug_rect_padding_px: int = 16

    aruco_dict: str = "DICT_4X4_1000"
    map_marker_ids: list[int] = field(default_factory=lambda: [0, 1, 2, 3])
    pinky_marker_ids: list[int] = field(default_factory=lambda: [14, 24, 34, 45, 54, 64])

    pinky_names: dict[int, str] = field(default_factory=dict)
    pinky_offsets_mm: dict[int, tuple[float, float]] = field(default_factory=dict)

    # 절대 회전 기준(전역 기준선). 핑키 yaw는 이 기준선 대비 상대각으로 계산.
    abs_heading_deg: float = 0.0
    abs_heading_p0_mm: Optional[tuple[float, float]] = None
    abs_heading_p1_mm: Optional[tuple[float, float]] = None

    ws_uri: str = "ws://127.0.0.1:8765"
    # WebSocket: 처리 프레임 기준 N프레임마다 1회 전송 (1=매 프레임)
    ws_send_every_n_frames: int = 5
    frame_id: str = "field_1880x1410"

    ui_filter_map_only: bool = False
    ui_filter_pinky_only: bool = False
    ui_debug_map_rect: bool = False
    ui_show_pos: bool = False
    ui_ws_send: bool = False
    ui_show_abs_heading_line: bool = False


def load_config(env_path: Path) -> AppConfig:
    vals = dotenv_values(env_path)
    cfg = AppConfig(env_path=env_path)

    cfg.camera_index = _as_int(vals.get("CAMERA_INDEX"), cfg.camera_index)
    cfg.camera_width = _as_int(vals.get("CAMERA_WIDTH"), cfg.camera_width)
    cfg.camera_height = _as_int(vals.get("CAMERA_HEIGHT"), cfg.camera_height)
    cfg.camera_flip_lr = _as_bool01(vals.get("CAMERA_FLIP_LR"), cfg.camera_flip_lr)
    cfg.camera_flip_ud = _as_bool01(vals.get("CAMERA_FLIP_UD"), cfg.camera_flip_ud)
    cfg.camera_target_fps = max(1, _as_int(vals.get("CAMERA_TARGET_FPS"), cfg.camera_target_fps))

    corners = list(default_map_corners_mm())
    for i in range(4):
        corners[i] = (
            _as_float(vals.get(f"MAP_CORNER_{i}_X_MM"), corners[i][0]),
            _as_float(vals.get(f"MAP_CORNER_{i}_Y_MM"), corners[i][1]),
        )
    cfg.map_corner_mm = corners

    cfg.map_center_to_camera_mm = _as_float(
        vals.get("MAP_CENTER_TO_CAMERA_MM"), cfg.map_center_to_camera_mm
    )
    cfg.map_debug_rect_padding_px = _as_int(
        vals.get("MAP_DEBUG_RECT_PADDING_PX"), cfg.map_debug_rect_padding_px
    )

    cfg.aruco_dict = _as_str(vals.get("ARUCO_DICT"), cfg.aruco_dict)
    cfg.map_marker_ids = _as_csv_ints(vals.get("MAP_MARKER_IDS"), cfg.map_marker_ids)
    cfg.pinky_marker_ids = _as_csv_ints(vals.get("PINKY_MARKER_IDS"), cfg.pinky_marker_ids)

    names = _parse_pinky_names(vals.get("PINKY_NAMES"))
    if names:
        cfg.pinky_names = names
    off = _parse_offsets_json(vals.get("PINKY_OFFSETS_JSON"))
    if off:
        cfg.pinky_offsets_mm = off

    cfg.abs_heading_deg = _as_float(vals.get("ABS_HEADING_DEG"), cfg.abs_heading_deg)
    p0x = vals.get("ABS_HEADING_P0_X_MM")
    p0y = vals.get("ABS_HEADING_P0_Y_MM")
    p1x = vals.get("ABS_HEADING_P1_X_MM")
    p1y = vals.get("ABS_HEADING_P1_Y_MM")
    try:
        if p0x is not None and p0y is not None and str(p0x).strip() != "" and str(p0y).strip() != "":
            cfg.abs_heading_p0_mm = (float(p0x), float(p0y))
    except Exception:
        cfg.abs_heading_p0_mm = None
    try:
        if p1x is not None and p1y is not None and str(p1x).strip() != "" and str(p1y).strip() != "":
            cfg.abs_heading_p1_mm = (float(p1x), float(p1y))
    except Exception:
        cfg.abs_heading_p1_mm = None

    cfg.ws_uri = _as_str(vals.get("WS_URI"), cfg.ws_uri)
    cfg.ws_send_every_n_frames = max(1, _as_int(vals.get("WS_SEND_EVERY_N_FRAMES"), cfg.ws_send_every_n_frames))
    cfg.frame_id = _as_str(vals.get("FRAME_ID"), cfg.frame_id)

    cfg.ui_filter_map_only = _as_bool01(vals.get("UI_FILTER_MAP_ONLY"), cfg.ui_filter_map_only)
    cfg.ui_filter_pinky_only = _as_bool01(vals.get("UI_FILTER_PINKY_ONLY"), cfg.ui_filter_pinky_only)
    cfg.ui_debug_map_rect = _as_bool01(vals.get("UI_DEBUG_MAP_RECT"), cfg.ui_debug_map_rect)
    cfg.ui_show_pos = _as_bool01(vals.get("UI_SHOW_POS"), cfg.ui_show_pos)
    cfg.ui_ws_send = _as_bool01(vals.get("UI_WS_SEND"), cfg.ui_ws_send)
    cfg.ui_show_abs_heading_line = _as_bool01(
        vals.get("UI_SHOW_ABS_HEADING_LINE"), cfg.ui_show_abs_heading_line
    )

    return cfg


def save_config(cfg: AppConfig) -> None:
    cfg.env_path.parent.mkdir(parents=True, exist_ok=True)
    if not cfg.env_path.exists():
        cfg.env_path.write_text("", encoding="utf-8")

    def put(k: str, v: str) -> None:
        set_key(str(cfg.env_path), k, v)

    put("CAMERA_INDEX", str(cfg.camera_index))
    put("CAMERA_WIDTH", str(cfg.camera_width))
    put("CAMERA_HEIGHT", str(cfg.camera_height))
    put("CAMERA_FLIP_LR", "1" if cfg.camera_flip_lr else "0")
    put("CAMERA_FLIP_UD", "1" if cfg.camera_flip_ud else "0")
    put("CAMERA_TARGET_FPS", str(cfg.camera_target_fps))

    for i, (xm, ym) in enumerate(cfg.map_corner_mm[:4]):
        put(f"MAP_CORNER_{i}_X_MM", str(xm))
        put(f"MAP_CORNER_{i}_Y_MM", str(ym))

    put("MAP_CENTER_TO_CAMERA_MM", str(cfg.map_center_to_camera_mm))
    put("MAP_DEBUG_RECT_PADDING_PX", str(cfg.map_debug_rect_padding_px))

    put("ARUCO_DICT", cfg.aruco_dict)
    put("MAP_MARKER_IDS", ",".join(map(str, cfg.map_marker_ids)))
    put("PINKY_MARKER_IDS", ",".join(map(str, cfg.pinky_marker_ids)))

    put("PINKY_NAMES", _pinky_names_to_str(cfg.pinky_names))
    put("PINKY_OFFSETS_JSON", _offsets_to_json(cfg.pinky_offsets_mm))
    put("ABS_HEADING_DEG", str(cfg.abs_heading_deg))
    put("ABS_HEADING_P0_X_MM", "" if cfg.abs_heading_p0_mm is None else str(cfg.abs_heading_p0_mm[0]))
    put("ABS_HEADING_P0_Y_MM", "" if cfg.abs_heading_p0_mm is None else str(cfg.abs_heading_p0_mm[1]))
    put("ABS_HEADING_P1_X_MM", "" if cfg.abs_heading_p1_mm is None else str(cfg.abs_heading_p1_mm[0]))
    put("ABS_HEADING_P1_Y_MM", "" if cfg.abs_heading_p1_mm is None else str(cfg.abs_heading_p1_mm[1]))

    put("WS_URI", cfg.ws_uri)
    put("WS_SEND_EVERY_N_FRAMES", str(cfg.ws_send_every_n_frames))
    put("FRAME_ID", cfg.frame_id)

    put("UI_FILTER_MAP_ONLY", "1" if cfg.ui_filter_map_only else "0")
    put("UI_FILTER_PINKY_ONLY", "1" if cfg.ui_filter_pinky_only else "0")
    put("UI_DEBUG_MAP_RECT", "1" if cfg.ui_debug_map_rect else "0")
    put("UI_SHOW_POS", "1" if cfg.ui_show_pos else "0")
    put("UI_WS_SEND", "1" if cfg.ui_ws_send else "0")
    put("UI_SHOW_ABS_HEADING_LINE", "1" if cfg.ui_show_abs_heading_line else "0")
