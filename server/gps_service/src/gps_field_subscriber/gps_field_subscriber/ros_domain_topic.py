# -*- coding: utf-8 -*-
"""Build topic path from ROS_DOMAIN_ID (namespace-style /pinky_{id}/gps_pos)."""

from __future__ import annotations

import os


def resolve_gps_topic(
    explicit_topic: str,
    pattern: str,
    *,
    pinky_id: int,
    logger=None,
) -> str:
    """If explicit_topic is non-empty, use it (leading / added if missing).

    Otherwise pattern.format(pinky_id=pinky_id).
    """
    exp = (explicit_topic or "").strip()
    if exp:
        return exp if exp.startswith("/") else f"/{exp}"

    # DDS는 ROS_DOMAIN_ID로 격리되지만, 토픽 문자열의 숫자는 pinky_id로 맞춘다.
    # {domain_id} 패턴도 레거시 호환으로 함께 채운다.
    try:
        t = pattern.format(pinky_id=int(pinky_id), domain_id=int(pinky_id))
    except Exception:
        t = f"/pinky_{int(pinky_id)}/gps_pos"
    return t if t.startswith("/") else f"/{t}"


def ros_domain_id_str() -> str:
    return os.environ.get("ROS_DOMAIN_ID", "?")
