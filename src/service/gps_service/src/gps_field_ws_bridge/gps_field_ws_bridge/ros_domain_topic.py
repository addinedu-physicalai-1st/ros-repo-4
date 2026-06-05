# -*- coding: utf-8 -*-
"""Same logic as gps_field_subscriber (avoid cross-package dependency)."""

from __future__ import annotations

import os


def resolve_gps_topic(
    explicit_topic: str,
    pattern: str,
    logger=None,
) -> str:
    exp = (explicit_topic or "").strip()
    if exp:
        return exp if exp.startswith("/") else f"/{exp}"

    raw = os.environ.get("ROS_DOMAIN_ID", "0")
    try:
        domain_id = int(raw)
    except ValueError:
        domain_id = 0
        if logger is not None:
            logger.warning(f"Invalid ROS_DOMAIN_ID={raw!r}, using domain_id=0")

    # 신규 패턴은 {pinky_id}. {domain_id}도 호환 유지.
    try:
        t = pattern.format(pinky_id=domain_id, domain_id=domain_id)
    except Exception:
        # pattern이 깨져도 최소한 기본 토픽으로 폴백
        t = f"/pinky_{domain_id}/gps_pos"
    return t if t.startswith("/") else f"/{t}"
