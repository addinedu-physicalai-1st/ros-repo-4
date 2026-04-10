# dalimi_GPS_GUI3: per-pinky pose (x_mm, y_mm, yaw_deg)

**Note:** This file name ends with `_KO`; the body is **English** for reliable UTF-8 in tooling. Korean explanation of the same content is provided in the project chat / copy from maintainer. One robot = one id in `PINKY_MARKER_IDS` = one ArUco marker id; four map markers build H (px→mm); then center_px→mm + offset, inside `map_corner_mm` polygon, yaw in mm vs `abs_heading_deg`; WebSocket one JSON per robot.

---

## 1. Pipeline (one frame)

1. Detect ArUco markers (`detect_aruco_markers_scaled`, optional downscale for speed).
2. Build **H** with **4 map marker IDs** and **4 mm corners** (`map_marker_ids`, `map_corner_mm`) via `solve_homography_from_4_markers_custom`. If any map ID is missing, **H = None** → no pinky coordinates.
3. For each `pid` in `pinky_marker_ids`, find first detection with `marker_id == pid`.
4. **Position:** `(x_mm, y_mm) = apply_homography(H, center_px) + pinky_offsets_mm[pid]`.
5. **Map gate:** keep only points inside the quadrilateral `map_corner_mm` (ray casting in mm).
6. **Yaw:** transform marker corners 0 and 1 to mm; `yaw_abs_deg = atan2(dy, dx)`; then `yaw_deg = yaw_rel` = relative to `abs_heading_deg` (wrapped to [-180, 180]).
7. Append dict to `robots` with `pinky_id`, optional `name`, `x_mm`, `y_mm`, `yaw_deg`, `yaw_abs_deg`.
8. **WebSocket:** if enabled, every `WS_SEND_EVERY_N_FRAMES` frames, `RobotWsSender` sends **one JSON per robot** (`x_mm`, `y_mm`, `yaw_deg`, `frame_id`, `pinky_id` as `pid & 0xFF`, `ros_main_id` as full `pid`).

Code reference: `src/dalimi_gps_gui3/ui/widgets/field_tab.py` (`_overlay`), `vision/aruco_utils.py`, `ws_sender.py`.

---

## 2. Map frame and homography

- `MAP_MARKER_IDS` (4 ids) and `MAP_CORNER_i_X_MM` / `MAP_CORNER_i_Y_MM` define **which pixel marker center maps to which mm corner**, in order.
- Default corners in code: `(0,0)`, `(1880,0)`, `(1880,1410)`, `(0,1410)` for ids `0,1,2,3`.

```80:99:dalimi_GPS_GUI3/src/dalimi_gps_gui3/vision/aruco_utils.py
def solve_homography_from_4_markers_custom(
    detections: list[MarkerDetection],
    ids_order: list[int],
    dst_corners_mm: list[tuple[float, float]],
) -> Optional[np.ndarray]:
    ...
    src = [got[i].center_px for i in ids_order]
    dst = [(float(x), float(y)) for x, y in dst_corners_mm]

    H, _mask = cv2.findHomography(
        np.array(src, dtype=np.float32), np.array(dst, dtype=np.float32)
    )
    return H
```

---

## 3. One pinky = one marker ID

Loop (simplified):

```412:440:dalimi_GPS_GUI3/src/dalimi_gps_gui3/ui/widgets/field_tab.py
            for pid in cfg.pinky_marker_ids:
                det = next((d for d in dets if d.marker_id == pid), None)
                if det is None:
                    continue
                x_mm, y_mm = apply_homography(H, det.center_px)
                off = cfg.pinky_offsets_mm.get(pid, (0.0, 0.0))
                x_mm += off[0]
                y_mm += off[1]
                if not _in_convex_quad((float(x_mm), float(y_mm)), poly_mm):
                    continue
                c0_mm = apply_homography(H, (det.corners_px[0][0], det.corners_px[0][1]))
                c1_mm = apply_homography(H, (det.corners_px[1][0], det.corners_px[1][1]))
                dx = float(c1_mm[0] - c0_mm[0])
                dy = float(c1_mm[1] - c0_mm[1])
                yaw_abs = float(math.degrees(math.atan2(dy, dx)))
                yaw_rel = float(((yaw_abs - float(cfg.abs_heading_deg) + 180.0) % 360.0) - 180.0)
                robots.append({...})
```

`PINKY_NAMES` is display metadata only; geometry uses **marker id** only.

---

## 4. WebSocket payload (per robot)

```78:90:dalimi_GPS_GUI3/src/dalimi_gps_gui3/ws_sender.py
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
                    payload = {k: v for k, v in payload.items() if v is not None}
                    await ws.send(json.dumps(payload, ensure_ascii=False))
```

---

## 5. Related `.env` keys

| Key | Role |
|-----|------|
| `MAP_MARKER_IDS` | Four map corner marker ids (order matches corners) |
| `MAP_CORNER_*_X_MM`, `MAP_CORNER_*_Y_MM` | mm coordinates for those ids |
| `PINKY_MARKER_IDS` | List of robot / pinky ArUco ids |
| `PINKY_OFFSETS_JSON` | Per-id mm offset `{ "14": [dx, dy], ... }` |
| `ABS_HEADING_DEG` | Global heading reference for `yaw_deg` |
| `WS_URI`, `FRAME_ID`, `WS_SEND_EVERY_N_FRAMES` | Send endpoint, frame id, send stride |

---

## 6. Caveats

- `next(...)` uses the **first** detection for that id; duplicate ids in view are unsafe.
- No lens distortion model in this path by default; mm error grows with lens / height.
- Map “inside” test is a simple polygon test on **mm** coordinates; corner order must match a consistent quadrilateral.

Path: `dalimi_GPS_GUI3/docs/PINKY_COORDINATES_KO.md`.
