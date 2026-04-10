from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

_ARUCO_DICTS: dict[str, int] = {
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
}


def get_aruco_dictionary(name: str) -> cv2.aruco.Dictionary:
    key = (name or "").strip()
    if key not in _ARUCO_DICTS:
        key = "DICT_4X4_1000"
    return cv2.aruco.getPredefinedDictionary(_ARUCO_DICTS[key])


@dataclass(frozen=True)
class MarkerDetection:
    marker_id: int
    corners_px: list[list[float]]
    center_px: tuple[float, float]


def detect_aruco_markers(
    bgr,
    dict_name: str,
    camera_matrix=None,
    dist_coeffs=None,
) -> list[MarkerDetection]:
    aruco_dict = get_aruco_dictionary(dict_name)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    corners, ids, _ = detector.detectMarkers(bgr)
    if ids is None or len(ids) == 0:
        return []

    out: list[MarkerDetection] = []
    for i, mid in enumerate(ids.flatten().tolist()):
        c = corners[i].reshape((4, 2)).astype(float).tolist()
        cx = sum(p[0] for p in c) / 4.0
        cy = sum(p[1] for p in c) / 4.0
        out.append(MarkerDetection(marker_id=int(mid), corners_px=c, center_px=(cx, cy)))
    return out


def draw_aruco_detections(bgr, detections: list[MarkerDetection], *, color=(0, 255, 0)) -> None:
    for d in detections:
        pts = d.corners_px
        for j in range(4):
            p1 = (int(pts[j][0]), int(pts[j][1]))
            p2 = (int(pts[(j + 1) % 4][0]), int(pts[(j + 1) % 4][1]))
            cv2.line(bgr, p1, p2, color, 2)
        cv2.circle(bgr, (int(d.center_px[0]), int(d.center_px[1])), 4, (0, 0, 255), -1)
        cv2.putText(
            bgr,
            f"ID {d.marker_id}",
            (int(d.center_px[0]) + 6, int(d.center_px[1]) - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 0, 0),
            2,
            cv2.LINE_AA,
        )


def solve_homography_from_4_markers_custom(
    detections: list[MarkerDetection],
    ids_order: list[int],
    dst_corners_mm: list[tuple[float, float]],
) -> Optional[np.ndarray]:
    """ids_order 길이 4, dst_corners_mm 동일 순서 (픽셀 중심 → mm)."""
    if len(ids_order) != 4 or len(dst_corners_mm) != 4:
        return None
    need = set(ids_order)
    got = {d.marker_id: d for d in detections if d.marker_id in need}
    if len(got) != 4:
        return None

    src = [got[i].center_px for i in ids_order]
    dst = [(float(x), float(y)) for x, y in dst_corners_mm]

    H, _mask = cv2.findHomography(
        np.array(src, dtype=np.float32), np.array(dst, dtype=np.float32)
    )
    return H


def detect_aruco_markers_scaled(
    bgr,
    dict_name: str,
    max_w: int = 1280,
) -> list[MarkerDetection]:
    """고해상도 프레임에서 저해상도로 검출 후 좌표를 원본 픽셀 기준으로 복원."""
    h, w = bgr.shape[:2]
    if w <= max_w:
        return detect_aruco_markers(bgr, dict_name)
    scale = max_w / float(w)
    nw = max_w
    nh = max(1, int(round(h * scale)))
    small = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_AREA)
    dets = detect_aruco_markers(small, dict_name)
    sx = w / float(nw)
    sy = h / float(nh)
    out: list[MarkerDetection] = []
    for d in dets:
        c = [[p[0] * sx, p[1] * sy] for p in d.corners_px]
        cx = d.center_px[0] * sx
        cy = d.center_px[1] * sy
        out.append(MarkerDetection(d.marker_id, c, (cx, cy)))
    return out


def apply_homography(H: np.ndarray, pt_px: tuple[float, float]) -> tuple[float, float]:
    x, y = float(pt_px[0]), float(pt_px[1])
    v = np.array([x, y, 1.0], dtype=np.float64).reshape((3, 1))
    w = H @ v
    if abs(float(w[2, 0])) < 1e-9:
        return float("nan"), float("nan")
    return float(w[0, 0] / w[2, 0]), float(w[1, 0] / w[2, 0])
