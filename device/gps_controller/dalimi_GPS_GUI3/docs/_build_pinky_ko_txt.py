# -*- coding: utf-8 -*-
"""ASCII-only: writes PINKY_COORDINATES_KO.txt with proper UTF-8 Korean."""
from pathlib import Path

def main() -> None:
    out = Path(__file__).resolve().parent / "PINKY_COORDINATES_KO.txt"
    # All Hangul via \uXXXX (BMP) — avoids editor/transport mojibake
    lines = [
        "dalimi_GPS_GUI3 - \ud551\ud0a4(\ub85c\ubd07)\ubcc4 \uc88c\ud45c\xb7\uc790\uc138\ub97c \uc5bb\ub294 \ubc29\uc2dd (\uc694\uc57d)",
        "=" * 64,
        "",
        "\u25a0 \uae30\ubcf8 \uac1c\ub150",
        '  - "\ud551\ud0a4 \ud558\ub098" = ArUco \ub9c8\ucee4 ID \ud558\ub098. \ud658\uacbd \ubcc0\uc218 PINKY_MARKER_IDS\uc5d0 \ub098\uc5f4\ud55c \uac01 ID\uc5d0 \ub300\ud574',
        "    \ub3d9\uc77c\ud55c \ubc29\uc2dd\uc73c\ub85c \uc88c\ud45c\ub97c \uacc4\uc0b0\ud55c\ub2e4. (ID\ub9cc \ub2e4\ub97c \ub550, \uc54c\uace0\ub9ac\uc998\uc740 \uacf5\ud1b5)",
        "  - \uae30\ud558 \uacc4\uc0b0\uc5d0\ub294 \ub9c8\ucee4 ID\ub9cc \uc0ac\uc6a9\ud55c\ub2e4. PINKY_NAMES\ub294 \ud654\uba74 \ud45c\uc2dc\uc6a9 \uc774\ub984\uc77c \ubfd0\uc774\ub2e4.",
        "",
        "\u25a0 \ud55c \ud504\ub808\uc784 \ucc98\ub9ac \uc21c\uc11c",
        "  1) \uce74\uba54\ub77c \uc601\uc0c1\uc5d0\uc11c ArUco \uac80\ucd9c (detect_aruco_markers_scaled \ub4f1, \ud544\uc694 \uc2dc \ub2e4\uc6b4\uc2a4\ucf00\uc77c \uac80\ucd9c)",
        "  2) \ub9f5 \uae30\uc900 \ubcc0\ud658: MAP_MARKER_IDS \uc21c\uc11c\uc640 MAP_CORNER_*_X_MM / Y_MM\uc73c\ub85c \uc8fc\uc5b4\uc9c4 \ub124 \ubaa8\uc11c\ub9ac\uc758 mm \uc88c\ud45c\ub97c \ub300\uc751\uc810\uc73c\ub85c \ud558\uc5ec",
        "     \ud638\ubaa8\uadf8\ub798\ud53c H\ub97c \uad6c\ud55c\ub2e4. (solve_homography_from_4_markers_custom)",
        "     \u2192 \ud53d\uc140 \uc88c\ud45c\ub97c \uacbd\uae30\uc7a5 \ubc14\ub2e5\uc758 \ud37c\ub9ac\ubbf8\ud130 mm \uc88c\ud45c\ub85c \uc62e\uae30\ub294 3x3 \ubcc0\ud658.",
        "     \ub124 \uac1c\uc758 \ub9f5 \ub9c8\ucee4 \uc911 \ud558\ub098\ub77c\ub3c4 \ud574\ub2f9 \ud504\ub808\uc784\uc5d0 \uc5c6\uc73c\uba74 H\uac00 \uc5c6\uace0, \ud551\ud0a4 \uc88c\ud45c\ub3c4 \uacc4\uc0b0\ud558\uc9c0 \uc54a\uc74c.",
        "  3) PINKY_MARKER_IDS\uc5d0 \uc788\ub294 \uac01 pid\uc5d0 \ub300\ud574:",
        "     - \uac80\ucd9c \ubaa9\ub85d\uc5d0\uc11c marker_id == pid \uc778 \uccab \ubc88\uc9f8 \uac80\ucd9c\ub9cc \uc0ac\uc6a9. (\uac19\uc740 ID\uac00 \uc5ec\ub7ec \uac1c\uba74 \uccab \uac80\ucd9c\ub9cc)",
        "     - \uc5c6\uc73c\uba74 \uadf8 \ud551\ud0a4\ub294 \ud574\ub2f9 \ud504\ub808\uc784 \uc2a4\ud0b5.",
        "  4) \uc704\uce58 (x_mm, y_mm):",
        "     - \uac80\ucd9c\ub41c \ub9c8\ucee4 \uc911\uc2ec det.center_px\uc5d0 H\ub97c \uc801\uc6a9 \u2192 mm\uc88c\ud45c",
        "     - \uc5ec\uae30\uc5d0 pinky_offsets_mm[pid] (\ub610\ub294 .env\uc758 PINKY_OFFSETS_JSON) \ub9cc\ud07c (dx, dy) \ub354\ud568",
        "       \u2192 \ub9c8\ucee4 \uc911\uc2ec\uacfc \uc2e4\uc81c \uae30\uc900\uc810(\ucc28\uccb4 \uc911\uc2ec \ub4f1) \ubcf4\uc815",
        "  5) \ub9f5 \uc601\uc5ed \ud544\ud130:",
        "     - map_corner_mm\uc73c\ub85c \ub9cc\ub4e0 \uc0ac\uac01\ud615(\ubcfc\ub85d \ub2e4\uac01\ud615) \uc548\uc5d0 (x_mm, y_mm)\uc774 \uc788\uc744 \ub54c\ub9cc \uc720\ud6a8.",
        "     - \ubc16\uc774\uba74 \ud45c\uc2dc\xb7\uc804\uc1a1 \ub300\uc0c1\uc5d0\uc11c \uc81c\uc678.",
        "  6) \ubc29\ud5a5 (yaw):",
        "     - \ub9c8\ucee4 \uad6c\uc11d\uc810 0\ubc88\xb71\ubc88\uc744 H\ub85c mm \uc88c\ud45c\ub85c \ubcc0\ud658, \uadf8 \ub450 \uc810\uc758 \ubca0\ud130\ub85c yaw_abs_deg = atan2(dy, dx) (\ub3c4)",
        "     - ABS_HEADING_DEG(\uc808\ub300 \uae30\uc900 \uac01)\uc744 \ube7c\uc11c \uc0c1\ub300\uac01 yaw_rel\ub85c \ubc14\uafb8\uace0,",
        "       UI\xb7WebSocket\uc758 yaw_deg\ub294 \uc774 \uc0c1\ub300\uac01(-180~180\ub3c4\ub85c \ub7a9).",
        "  7) robots \ub9ac\uc2a4\ud2b8\uc5d0 pinky_id, name(\uc120\ud0dd), x_mm, y_mm, yaw_deg, yaw_abs_deg \uc800\uc7a5.",
        "",
        "\u25a0 WebSocket \uc804\uc1a1",
        "  - UI\uc5d0\uc11c WebSocket \uc804\uc1a1\uc774 \ucf1c\uc838 \uc788\uace0, WS_SEND_EVERY_N_FRAMES\ub9c8\ub2e4",
        "    \ub85c\ubd07(\ud551\ud0a4)\ub9c8\ub2e4 JSON \ud55c \uac74\uc529 \uc804\uc1a1: x_mm, y_mm, yaw_deg, frame_id,",
        "    pinky_id(pid\uc758 \ud558\uc704 8\ube44\ud2b8), ros_main_id(\uc804\uccb4 pid).",
        "",
        "\u25a0 \uad00\ub828 .env \ud0a4",
        "  - MAP_MARKER_IDS         : \ub9f5 \ubaa8\uc11c\ub9ac \ub123\uc7404\uac1c\uc5d0 \ub300\uc751\ud558\ub294 \ub9c8\ucee4 ID (\uc21c\uc11c = \ubaa8\uc11c\ub9ac \ub123\uc740 \uc21c\uc11c)",
        "  - MAP_CORNER_i_X_MM, Y_MM : \uac01 ID\uac00 \ub300\uc751\ud558\ub294 mm\uc88c\ud45c",
        "  - PINKY_MARKER_IDS        : \ud551\ud0a4(\ub85c\ubd07) ArUco ID \ubaa9\ub85d",
        "  - PINKY_OFFSETS_JSON      : ID\ubcc4 mm \uc624\ud504\uc14b JSON, \uc608: {\"14\": [dx, dy], ...}",
        "  - PINKY_NAMES             : \ud45c\uc2dc\uc6a9 \uc774\ub984\ub9cc (\uc88c\ud45c \uacc4\uc0b0\uacfc \ubb34\uad00)",
        "  - ABS_HEADING_DEG         : yaw \uc0c1\ub300\uac01 \uae30\uc900",
        "  - WS_URI, FRAME_ID, WS_SEND_EVERY_N_FRAMES : \uc804\uc1a1 \uc124\uc815",
        "",
        "\u25a0 \uc8fc\uc758\uc0ac\ud56d",
        "  - \ub3d9\uc77c \ub9c8\ucee4 ID\uac00 \ud654\uba74\uc5d0 \ub450 \ubc88 \ub098\uc624\uba74 \uccab \uac80\ucd9c\ub9cc \uc4f0\uc5ec \uc798\ubabb\ub41c \uc704\uce58\uac00 \ub420 \uc218 \uc788\uc74c.",
        "  - \uae30\ubcf8 \uacbd\ub85c\uc5d0\uc11c\ub294 \ub80c\uc988 \uc678\uace1 \ubcf4\uc815\uc744 \uc4f0\uc9c0 \uc54a\uc744 \uc218 \uc788\uc5b4, \ub80c\uc988\xb7\ub192\uc774\uc5d0 \ub530\ub77c mm \uc624\ucc28\uac00 \uce60\uc218\ub85d \ucee4\uc9d0.",
        "  - \ub9f5 \uc548\ucabd \ud310\uc815\uc740 mm\ud3c9\uba74\uc5d0\uc11c \ub2e8\uc21c \ub2e4\uac01\ud615 \ud3ec\ud568 \uac80\uc0ac\uc774\uba70, \ubaa8\uc11c\ub9ac \ub123\uc740 \uc21c\uc11c\uac00",
        "    \uc77c\uad00\ub41c \uc0ac\uac01\ud615\uc774\uc5b4\uc57c \ud55c\ub2e4.",
        "",
        "\u25a0 \ucf54\ub4dc \uc704\uce58",
        "  - src/dalimi_gps_gui3/ui/widgets/field_tab.py (_overlay: \uac80\ucd9c\xb7\ud638\ubaa8\uadf8\ub798\ud53c\xb7yaw)",
        "  - src/dalimi_gps_gui3/vision/aruco_utils.py (\uac80\ucd9c, \ud638\ubaa8\uadf8\ub798\ud53c, apply_homography)",
        "  - src/dalimi_gps_gui3/ws_sender.py (WebSocket \ud398\uc774\ub85c\ub4dc)",
        "  - src/dalimi_gps_gui3/config.py (load_config\uc5d0\uc11c .env \ubc18\uc601)",
        "",
        "\ud30c\uc77c \uacbd\ub85c: dalimi_GPS_GUI3/docs/PINKY_COORDINATES_KO.txt",
    ]
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("Wrote", out, "UTF-8 OK")


if __name__ == "__main__":
    main()
