# -*- coding: utf-8 -*-
"""Same as subscriber (no cross-package import)."""

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

    t = pattern.format(domain_id=domain_id)
    return t if t.startswith("/") else f"/{t}"
