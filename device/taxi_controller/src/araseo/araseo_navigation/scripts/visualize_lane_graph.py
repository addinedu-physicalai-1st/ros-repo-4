#!/usr/bin/env python3
"""Visualize the directed lane graph described in lane_graph.yaml.

The script draws each lane's centerline, corridor width, waypoint labels, lane
IDs, and successor connections so it is easy to understand how the road graph
is composed.
"""

from __future__ import annotations

import argparse
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import yaml


@dataclass(frozen=True)
class Lane:
    lane_id: str
    centerline: list[tuple[float, float]]
    successors: list[str]
    width: float


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[1]
    default_graph = repo_root / "params" / "lane_graph.yaml"

    parser = argparse.ArgumentParser(
        description="Visualize the lane graph in pinky_navigation/params/lane_graph.yaml"
    )
    parser.add_argument(
        "--graph",
        default=str(default_graph),
        help=f"Path to lane_graph.yaml (default: {default_graph})",
    )
    parser.add_argument(
        "--output",
        default="",
        help="Optional path to save the figure instead of only showing it",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=200,
        help="Figure DPI when saving or showing (default: 200)",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not open an interactive window",
    )
    parser.add_argument(
        "--hide-point-labels",
        action="store_true",
        help="Hide individual waypoint coordinate labels",
    )
    parser.add_argument(
        "--hide-lane-labels",
        action="store_true",
        help="Hide lane ID labels",
    )
    parser.add_argument(
        "--hide-successors",
        action="store_true",
        help="Hide successor arrows between lanes",
    )
    return parser.parse_args()


def load_graph(graph_path: Path) -> tuple[float, dict[str, Lane]]:
    with graph_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict):
        raise ValueError("lane graph YAML must contain a top-level mapping")

    default_width = float(data.get("default_lane_width", 0.16))
    lanes_node = data.get("lanes")
    if not isinstance(lanes_node, list):
        raise ValueError("lane graph YAML must contain a top-level 'lanes' sequence")

    lanes: dict[str, Lane] = {}
    for lane_node in lanes_node:
        if not isinstance(lane_node, dict):
            raise ValueError("each lane entry must be a mapping")

        lane_id = str(lane_node.get("id", "")).strip()
        if not lane_id:
            raise ValueError("each lane must have a non-empty 'id'")

        centerline_node = lane_node.get("centerline")
        if not isinstance(centerline_node, list) or len(centerline_node) < 2:
            raise ValueError(f"lane '{lane_id}' must have at least two centerline points")

        centerline: list[tuple[float, float]] = []
        for point_node in centerline_node:
            if not isinstance(point_node, (list, tuple)) or len(point_node) != 2:
                raise ValueError(f"lane '{lane_id}' contains an invalid centerline point")
            centerline.append((float(point_node[0]), float(point_node[1])))

        successors_node = lane_node.get("successors", [])
        if not isinstance(successors_node, list):
            raise ValueError(f"lane '{lane_id}' successors must be a sequence")
        successors = [str(item) for item in successors_node]

        lane_width = float(lane_node.get("width", default_width))
        lanes[lane_id] = Lane(
            lane_id=lane_id,
            centerline=centerline,
            successors=successors,
            width=lane_width,
        )

    for lane in lanes.values():
        for successor in lane.successors:
            if successor not in lanes:
                raise ValueError(
                    f"lane '{lane.lane_id}' references unknown successor '{successor}'"
                )

    return default_width, lanes


def segment_length(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def lane_length(centerline: list[tuple[float, float]]) -> float:
    return sum(segment_length(centerline[i - 1], centerline[i]) for i in range(1, len(centerline)))


def normalize(vx: float, vy: float) -> tuple[float, float]:
    norm = math.hypot(vx, vy)
    if norm == 0.0:
        return 0.0, 0.0
    return vx / norm, vy / norm


def perpendicular(a: tuple[float, float], b: tuple[float, float]) -> tuple[float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    nx, ny = normalize(-dy, dx)
    return nx, ny


def lane_segments(centerline: list[tuple[float, float]]) -> Iterable[tuple[tuple[float, float], tuple[float, float]]]:
    for i in range(1, len(centerline)):
        yield centerline[i - 1], centerline[i]


def lane_corridor_polygons(
    centerline: list[tuple[float, float]],
    width: float,
) -> list[list[tuple[float, float]]]:
    half = width / 2.0
    polygons: list[list[tuple[float, float]]] = []
    for start, end in lane_segments(centerline):
        nx, ny = perpendicular(start, end)
        polygons.append(
            [
                (start[0] + nx * half, start[1] + ny * half),
                (start[0] - nx * half, start[1] - ny * half),
                (end[0] - nx * half, end[1] - ny * half),
                (end[0] + nx * half, end[1] + ny * half),
            ]
        )
    return polygons


def midpoint(a: tuple[float, float], b: tuple[float, float]) -> tuple[float, float]:
    return ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)


def curved_successor_label_offset(index: int) -> float:
    return 0.02 if index % 2 == 0 else -0.02


def build_plot(default_width: float, lanes: dict[str, Lane], args: argparse.Namespace) -> None:
    import matplotlib

    if args.output or not os.environ.get("DISPLAY"):
        matplotlib.use("Agg")

    import matplotlib.pyplot as plt
    from matplotlib.patches import FancyArrowPatch, Polygon

    fig, ax = plt.subplots(figsize=(10, 8), constrained_layout=True)
    ax.set_title(
        f"Pinky lane graph visualization  |  default width = {default_width:.2f} m",
        fontsize=14,
        pad=14,
    )
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True, linestyle="--", alpha=0.25)

    # Draw broad corridors first, then centerlines and labels on top.
    for lane in lanes.values():
        for polygon in lane_corridor_polygons(lane.centerline, lane.width):
            ax.add_patch(
                Polygon(
                    polygon,
                    closed=True,
                    facecolor="#8bd3c7",
                    edgecolor="#2c7f7b",
                    linewidth=0.8,
                    alpha=0.22,
                    zorder=1,
                )
            )

    for lane in lanes.values():
        xs = [p[0] for p in lane.centerline]
        ys = [p[1] for p in lane.centerline]
        ax.plot(xs, ys, color="#263238", linewidth=2.0, zorder=3)
        ax.scatter(xs, ys, s=18, color="#ffffff", edgecolors="#263238", linewidths=0.8, zorder=4)

        if not args.hide_lane_labels:
            total = lane_length(lane.centerline)
            mid_idx = len(lane.centerline) // 2
            label_x, label_y = lane.centerline[mid_idx]
            ax.text(
                label_x,
                label_y,
                f"{lane.lane_id}\n{total:.2f} m",
                fontsize=8,
                ha="center",
                va="center",
                color="#102027",
                bbox=dict(boxstyle="round,pad=0.2", facecolor="#fffde7", edgecolor="#c9b458", alpha=0.9),
                zorder=6,
            )

        if not args.hide_point_labels:
            for i, (x, y) in enumerate(lane.centerline):
                ax.text(
                    x + 0.01,
                    y + 0.01,
                    f"({x:.2f}, {y:.2f})",
                    fontsize=7,
                    color="#37474f",
                    zorder=5,
                )

        # Direction arrow on the longest visible segment near the middle.
        seg_idx = max(0, len(lane.centerline) // 2 - 1)
        start = lane.centerline[seg_idx]
        end = lane.centerline[min(seg_idx + 1, len(lane.centerline) - 1)]
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        ax.add_patch(
            FancyArrowPatch(
                posA=(start[0], start[1]),
                posB=(end[0], end[1]),
                arrowstyle="-|>",
                mutation_scale=14,
                linewidth=1.4,
                color="#ff8f00",
                zorder=5,
            )
        )

    if not args.hide_successors:
        seen_edges: set[tuple[str, str]] = set()
        for lane in lanes.values():
            src = lane.centerline[-1]
            for i, successor_id in enumerate(lane.successors):
                edge = (lane.lane_id, successor_id)
                if edge in seen_edges:
                    continue
                seen_edges.add(edge)

                successor = lanes[successor_id]
                dst = successor.centerline[0]
                mx, my = midpoint(src, dst)
                offset = curved_successor_label_offset(i)
                ax.add_patch(
                    FancyArrowPatch(
                        posA=src,
                        posB=dst,
                        arrowstyle="->",
                        mutation_scale=12,
                        linewidth=1.2,
                        linestyle="--",
                        color="#ef6c00",
                        alpha=0.75,
                        connectionstyle=f"arc3,rad={offset}",
                        zorder=2,
                    )
                )
                ax.text(
                    mx,
                    my + offset * 0.6,
                    f"{lane.lane_id} -> {successor_id}",
                    fontsize=6,
                    color="#bf360c",
                    ha="center",
                    va="center",
                    zorder=6,
                )

    all_x = [x for lane in lanes.values() for x, _ in lane.centerline]
    all_y = [y for lane in lanes.values() for _, y in lane.centerline]
    margin = 0.10
    ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
    ax.set_ylim(min(all_y) - margin, max(all_y) + margin)

    legend_handles = [
        plt.Line2D([0], [0], color="#263238", linewidth=2.0, label="centerline"),
        plt.Line2D([0], [0], color="#8bd3c7", linewidth=8, alpha=0.22, label="corridor"),
        plt.Line2D([0], [0], color="#ff8f00", linewidth=1.5, label="direction"),
        plt.Line2D([0], [0], color="#ef6c00", linewidth=1.5, linestyle="--", label="successor"),
    ]
    ax.legend(handles=legend_handles, loc="upper right", framealpha=0.95)

    output_path = args.output.strip()
    if output_path:
        fig.savefig(output_path, dpi=args.dpi)
        print(f"saved: {output_path}")

    if not args.no_show and not output_path:
        plt.show()
    plt.close(fig)


def main() -> int:
    args = parse_args()
    graph_path = Path(args.graph).expanduser().resolve()
    if not graph_path.exists():
        raise FileNotFoundError(f"lane graph not found: {graph_path}")

    default_width, lanes = load_graph(graph_path)
    print(f"loaded {len(lanes)} lanes from {graph_path}")
    print(f"default lane width: {default_width:.2f} m")

    build_plot(default_width, lanes, args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
