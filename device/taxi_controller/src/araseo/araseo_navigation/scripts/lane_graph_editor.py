#!/usr/bin/env python3
"""Interactive editor for araseo_navigation lane_graph.yaml files."""

from __future__ import annotations

import argparse
import math
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path

import yaml
from PyQt5 import QtCore, QtGui, QtWidgets, uic


SCENE_SCALE = 420.0
POINT_RADIUS = 6.0
LANE_KIND_OPTIONS = ["road", "connector"]
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
INSPECTOR_UI_PATH = PACKAGE_ROOT / "ui" / "lane_graph_editor_inspector.ui"


def world_to_scene(x: float, y: float) -> QtCore.QPointF:
    return QtCore.QPointF(x * SCENE_SCALE, -y * SCENE_SCALE)


def scene_to_world(pos: QtCore.QPointF) -> tuple[float, float]:
    return pos.x() / SCENE_SCALE, -pos.y() / SCENE_SCALE


def format_coord(value: float) -> str:
    return f"{value:.3f}"


@dataclass
class LaneData:
    lane_id: str
    kind: str
    width: float
    centerline: list[tuple[float, float]]
    successors: list[str] = field(default_factory=list)
    extras: dict = field(default_factory=dict)


@dataclass
class LaneDocument:
    path: Path
    top_level: dict
    default_lane_width: float
    lanes: list[LaneData]
    next_lane_number: int = 1

    @classmethod
    def load(cls, path: Path) -> "LaneDocument":
        with path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream)

        if not isinstance(data, dict):
            raise ValueError("lane graph YAML must contain a top-level mapping")

        lanes_node = data.get("lanes")
        if not isinstance(lanes_node, list):
            raise ValueError("lane graph YAML must contain a top-level 'lanes' list")

        default_lane_width = float(data.get("default_lane_width", 0.16))
        lanes: list[LaneData] = []
        max_number = 0

        for lane_node in lanes_node:
            if not isinstance(lane_node, dict):
                raise ValueError("each lane must be a mapping")

            lane_id = str(lane_node.get("id", "")).strip()
            if not lane_id:
                raise ValueError("each lane must have a non-empty id")

            centerline_node = lane_node.get("centerline")
            if not isinstance(centerline_node, list) or len(centerline_node) < 2:
                raise ValueError(f"lane '{lane_id}' must contain at least two centerline points")

            centerline: list[tuple[float, float]] = []
            for point_node in centerline_node:
                if not isinstance(point_node, (list, tuple)) or len(point_node) < 2:
                    raise ValueError(f"lane '{lane_id}' has an invalid centerline point")
                centerline.append((float(point_node[0]), float(point_node[1])))

            successors = lane_node.get("successors", [])
            if not isinstance(successors, list):
                raise ValueError(f"lane '{lane_id}' successors must be a list")

            lanes.append(
                LaneData(
                    lane_id=lane_id,
                    kind=str(lane_node.get("kind", "road")),
                    width=float(lane_node.get("width", default_lane_width)),
                    centerline=centerline,
                    successors=[str(item) for item in successors],
                    extras={
                        key: value
                        for key, value in lane_node.items()
                        if key not in {"id", "kind", "centerline", "successors", "width"}
                    },
                )
            )

            if lane_id.startswith("lane_"):
                suffix = lane_id.split("_", 1)[1]
                if suffix.isdigit():
                    max_number = max(max_number, int(suffix))

        document = cls(
            path=path,
            top_level={key: value for key, value in data.items() if key != "lanes"},
            default_lane_width=default_lane_width,
            lanes=lanes,
            next_lane_number=max_number + 1,
        )
        document.validate()
        return document

    def find_lane(self, lane_id: str) -> LaneData | None:
        for lane in self.lanes:
            if lane.lane_id == lane_id:
                return lane
        return None

    def generate_lane_id(self) -> str:
        while True:
            lane_id = f"lane_{self.next_lane_number}"
            self.next_lane_number += 1
            if self.find_lane(lane_id) is None:
                return lane_id

    def remove_lane(self, lane_id: str) -> None:
        self.lanes = [lane for lane in self.lanes if lane.lane_id != lane_id]
        for lane in self.lanes:
            lane.successors = [successor for successor in lane.successors if successor != lane_id]

    def rename_lane(self, old_id: str, new_id: str) -> None:
        lane = self.find_lane(old_id)
        if lane is None:
            raise ValueError(f"unknown lane '{old_id}'")
        if new_id != old_id and self.find_lane(new_id) is not None:
            raise ValueError(f"duplicate lane id: {new_id}")
        lane.lane_id = new_id
        for other_lane in self.lanes:
            other_lane.successors = [new_id if item == old_id else item for item in other_lane.successors]

    def validate(self) -> None:
        seen_ids: set[str] = set()
        valid_ids = {lane.lane_id for lane in self.lanes}
        for lane in self.lanes:
            if not lane.lane_id:
                raise ValueError("lane id cannot be empty")
            if lane.lane_id in seen_ids:
                raise ValueError(f"duplicate lane id: {lane.lane_id}")
            seen_ids.add(lane.lane_id)
            if len(lane.centerline) < 2:
                raise ValueError(f"lane '{lane.lane_id}' must have at least two points")
            if lane.kind not in LANE_KIND_OPTIONS:
                raise ValueError(f"lane '{lane.lane_id}' kind must be one of {LANE_KIND_OPTIONS}")
            for successor in lane.successors:
                if successor not in valid_ids:
                    raise ValueError(
                        f"lane '{lane.lane_id}' references unknown successor '{successor}'"
                    )

    def serialize(self) -> dict:
        root = dict(self.top_level)
        if "default_lane_width" in self.top_level or self.default_lane_width != 0.16:
            root["default_lane_width"] = self.default_lane_width

        root["lanes"] = []
        for lane in self.lanes:
            lane_node = {
                "id": lane.lane_id,
                "kind": lane.kind,
                "centerline": [[round(x, 6), round(y, 6)] for x, y in lane.centerline],
                "successors": list(lane.successors),
            }
            if lane.width != self.default_lane_width:
                lane_node["width"] = lane.width
            lane_node.update(lane.extras)
            root["lanes"].append(lane_node)
        return root

    def save(self) -> None:
        self.validate()
        payload = self.serialize()
        with tempfile.NamedTemporaryFile(
            "w", encoding="utf-8", dir=str(self.path.parent), delete=False
        ) as tmp:
            yaml.safe_dump(payload, tmp, sort_keys=False, allow_unicode=False)
            temp_path = Path(tmp.name)
        temp_path.replace(self.path)

    def apply_transform(
        self, scale_x: float, scale_y: float, origin_x: float, origin_y: float
    ) -> None:
        for lane in self.lanes:
            lane.centerline = [
                (x * scale_x + origin_x, y * scale_y + origin_y)
                for x, y in lane.centerline
            ]


@dataclass
class EditorSession:
    mode: str = "select"
    dirty: bool = False
    selected_lane_id: str | None = None
    selected_point_index: int | None = None
    pending_lane_points: list[tuple[float, float]] = field(default_factory=list)
    pending_successor_source: str | None = None


class EditorModel(QtCore.QObject):
    changed = QtCore.pyqtSignal()
    selection_changed = QtCore.pyqtSignal()
    mode_changed = QtCore.pyqtSignal()
    error_raised = QtCore.pyqtSignal(str)
    save_succeeded = QtCore.pyqtSignal(str)

    def __init__(self, document: LaneDocument) -> None:
        super().__init__()
        self.document = document
        self.session = EditorSession()

    def selected_lane(self) -> LaneData | None:
        if self.session.selected_lane_id is None:
            return None
        return self.document.find_lane(self.session.selected_lane_id)

    def selected_point(self) -> tuple[int, tuple[float, float]] | None:
        lane = self.selected_lane()
        point_index = self.session.selected_point_index
        if lane is None or point_index is None:
            return None
        if not (0 <= point_index < len(lane.centerline)):
            self.session.selected_point_index = None
            return None
        return point_index, lane.centerline[point_index]

    def mark_dirty(self) -> None:
        if self.session.dirty:
            return
        self.session.dirty = True
        self.changed.emit()

    def set_mode(self, mode: str) -> None:
        self.session.mode = mode
        if mode != "create":
            self.session.pending_lane_points.clear()
        if mode != "successor":
            self.session.pending_successor_source = None
        self.mode_changed.emit()
        self.changed.emit()

    def cancel_active_mode(self) -> None:
        self.session.pending_lane_points.clear()
        self.session.pending_successor_source = None
        self.set_mode("select")

    def select_lane(self, lane_id: str | None) -> None:
        self.session.selected_lane_id = lane_id
        self.session.selected_point_index = None
        self.selection_changed.emit()
        self.changed.emit()

    def select_point(self, lane_id: str | None, point_index: int | None) -> None:
        self.session.selected_lane_id = lane_id
        self.session.selected_point_index = point_index
        self.selection_changed.emit()
        self.changed.emit()

    def update_lane_properties(self, lane_id: str, new_id: str, kind: str, width: float) -> bool:
        new_id = new_id.strip()
        if not new_id:
            self.error_raised.emit("Lane ID cannot be empty.")
            return False
        if kind not in LANE_KIND_OPTIONS:
            self.error_raised.emit(f"Lane kind must be one of {LANE_KIND_OPTIONS}.")
            return False

        try:
            if new_id != lane_id:
                self.document.rename_lane(lane_id, new_id)
                self.session.selected_lane_id = new_id
            lane = self.selected_lane()
            if lane is None:
                return False
            lane.kind = kind
            lane.width = width
            self.document.validate()
        except ValueError as exc:
            self.error_raised.emit(str(exc))
            return False

        self.mark_dirty()
        self.selection_changed.emit()
        self.changed.emit()
        return True

    def update_point(self, lane_id: str, point_index: int, x: float, y: float) -> bool:
        lane = self.document.find_lane(lane_id)
        if lane is None or not (0 <= point_index < len(lane.centerline)):
            return False
        lane.centerline[point_index] = (x, y)
        self.session.selected_lane_id = lane_id
        self.session.selected_point_index = point_index
        self.mark_dirty()
        self.selection_changed.emit()
        self.changed.emit()
        return True

    def add_create_point(self, x: float, y: float) -> None:
        self.session.pending_lane_points.append((x, y))
        self.changed.emit()

    def finish_pending_lane(self) -> bool:
        if len(self.session.pending_lane_points) < 2:
            self.error_raised.emit("A lane needs at least two points.")
            return False
        lane = LaneData(
            lane_id=self.document.generate_lane_id(),
            kind="road",
            width=self.document.default_lane_width,
            centerline=list(self.session.pending_lane_points),
            successors=[],
        )
        self.document.lanes.append(lane)
        self.session.pending_lane_points.clear()
        self.session.selected_lane_id = lane.lane_id
        self.session.selected_point_index = None
        self.mark_dirty()
        self.selection_changed.emit()
        self.changed.emit()
        return True

    def start_successor(self, lane_id: str) -> None:
        self.session.pending_successor_source = lane_id
        self.session.selected_lane_id = lane_id
        self.session.selected_point_index = None
        self.selection_changed.emit()
        self.changed.emit()

    def connect_successor(self, target_lane_id: str) -> bool:
        source_lane_id = self.session.pending_successor_source
        if source_lane_id is None:
            self.start_successor(target_lane_id)
            return False

        source_lane = self.document.find_lane(source_lane_id)
        target_lane = self.document.find_lane(target_lane_id)
        if source_lane is None or target_lane is None:
            self.session.pending_successor_source = None
            self.changed.emit()
            return False
        if source_lane.lane_id == target_lane.lane_id:
            self.error_raised.emit("Cannot connect a lane to itself.")
            return False
        if target_lane.lane_id not in source_lane.successors:
            source_lane.successors.append(target_lane.lane_id)
            self.mark_dirty()

        self.session.pending_successor_source = None
        self.session.selected_lane_id = source_lane.lane_id
        self.session.selected_point_index = None
        self.selection_changed.emit()
        self.changed.emit()
        return True

    def remove_selected_successor(self, successor_index: int) -> bool:
        lane = self.selected_lane()
        if lane is None or not (0 <= successor_index < len(lane.successors)):
            return False
        lane.successors.pop(successor_index)
        self.mark_dirty()
        self.changed.emit()
        return True

    def delete_selected_lane(self) -> bool:
        lane_id = self.session.selected_lane_id
        if lane_id is None:
            return False
        self.document.remove_lane(lane_id)
        self.session.selected_lane_id = None
        self.session.selected_point_index = None
        self.mark_dirty()
        self.selection_changed.emit()
        self.changed.emit()
        return True

    def apply_transform(
        self, scale_x: float, scale_y: float, origin_x: float, origin_y: float
    ) -> None:
        self.document.apply_transform(scale_x, scale_y, origin_x, origin_y)
        self.mark_dirty()
        self.changed.emit()

    def save(self) -> bool:
        try:
            self.document.save()
        except Exception as exc:
            self.error_raised.emit(str(exc))
            return False
        self.session.dirty = False
        self.changed.emit()
        self.save_succeeded.emit("Saved.")
        return True


class ArrowPathItem(QtWidgets.QGraphicsPathItem):
    def __init__(self, pen: QtGui.QPen, parent: QtWidgets.QGraphicsItem | None = None) -> None:
        super().__init__(parent)
        self.setPen(pen)
        self.arrow_head = QtWidgets.QGraphicsPolygonItem(self)
        self.arrow_head.setBrush(pen.color())
        self.arrow_head.setPen(QtGui.QPen(pen.color()))

    def set_arrow_path(self, path: QtGui.QPainterPath, end_angle_radians: float) -> None:
        self.setPath(path)
        end_point = path.pointAtPercent(1.0)
        head_size = 10.0
        left = end_point + QtCore.QPointF(
            math.cos(end_angle_radians + math.pi * 0.85) * head_size,
            math.sin(end_angle_radians + math.pi * 0.85) * head_size,
        )
        right = end_point + QtCore.QPointF(
            math.cos(end_angle_radians - math.pi * 0.85) * head_size,
            math.sin(end_angle_radians - math.pi * 0.85) * head_size,
        )
        self.arrow_head.setPolygon(QtGui.QPolygonF([end_point, left, right]))


class LanePointItem(QtWidgets.QGraphicsEllipseItem):
    def __init__(self, lane_id: str, index: int, controller: "EditorController") -> None:
        super().__init__(-POINT_RADIUS, -POINT_RADIUS, POINT_RADIUS * 2, POINT_RADIUS * 2)
        self.lane_id = lane_id
        self.index = index
        self.controller = controller
        self._syncing_position = False
        self.setBrush(QtGui.QBrush(QtGui.QColor("#ffffff")))
        self.setPen(QtGui.QPen(QtGui.QColor("#263238"), 1.2))
        self.setFlag(QtWidgets.QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QtWidgets.QGraphicsItem.ItemIsSelectable, True)
        self.setFlag(QtWidgets.QGraphicsItem.ItemSendsGeometryChanges, True)
        self.setZValue(20)

    def mousePressEvent(self, event: QtWidgets.QGraphicsSceneMouseEvent) -> None:
        self.controller.select_point(self.lane_id, self.index)
        super().mousePressEvent(event)

    def itemChange(self, change: "QtWidgets.QGraphicsItem.GraphicsItemChange", value):
        if change == QtWidgets.QGraphicsItem.ItemPositionChange and not self._syncing_position:
            x, y = scene_to_world(value)
            self.controller.move_point(self.lane_id, self.index, x, y)
        return super().itemChange(change, value)


class LaneItem:
    def __init__(self, lane_id: str, controller: "EditorController") -> None:
        self.lane_id = lane_id
        self.controller = controller
        self.path_item = ArrowPathItem(QtGui.QPen(QtGui.QColor("#1f2933"), 2.4))
        self.path_item.setFlag(QtWidgets.QGraphicsItem.ItemIsSelectable, True)
        self.path_item.setZValue(5)
        self.path_item.setData(0, lane_id)
        self.label_item = QtWidgets.QGraphicsSimpleTextItem(lane_id)
        self.label_item.setFlag(QtWidgets.QGraphicsItem.ItemIgnoresTransformations, True)
        self.label_item.setBrush(QtGui.QBrush(QtGui.QColor("#0f172a")))
        self.label_item.setVisible(False)
        self.label_item.setZValue(30)
        self.label_item.setData(0, lane_id)
        self.point_items: list[LanePointItem] = []
        self.successor_items: list[ArrowPathItem] = []

    def add_to_scene(self, scene: QtWidgets.QGraphicsScene) -> None:
        scene.addItem(self.path_item)
        scene.addItem(self.label_item)
        for point_item in self.point_items:
            scene.addItem(point_item)

    def remove_from_scene(self, scene: QtWidgets.QGraphicsScene) -> None:
        for successor_item in self.successor_items:
            if successor_item.scene():
                scene.removeItem(successor_item)
        for point_item in self.point_items:
            if point_item.scene():
                scene.removeItem(point_item)
        if self.path_item.scene():
            scene.removeItem(self.path_item)
        if self.label_item.scene():
            scene.removeItem(self.label_item)

    def sync_from_lane(self, lane: LaneData, scene: QtWidgets.QGraphicsScene) -> None:
        self.lane_id = lane.lane_id
        self.path_item.setData(0, lane.lane_id)
        self.label_item.setData(0, lane.lane_id)
        self.label_item.setText(lane.lane_id)

        while len(self.point_items) < len(lane.centerline):
            point_item = LanePointItem(lane.lane_id, len(self.point_items), self.controller)
            self.point_items.append(point_item)
            if self.path_item.scene():
                scene.addItem(point_item)

        while len(self.point_items) > len(lane.centerline):
            point_item = self.point_items.pop()
            if point_item.scene():
                scene.removeItem(point_item)

        for index, (x, y) in enumerate(lane.centerline):
            point_item = self.point_items[index]
            point_item.index = index
            point_item.lane_id = lane.lane_id
            point_item._syncing_position = True
            point_item.setPos(world_to_scene(x, y))
            point_item._syncing_position = False

        path = QtGui.QPainterPath()
        start = world_to_scene(*lane.centerline[0])
        path.moveTo(start)
        for point in lane.centerline[1:]:
            path.lineTo(world_to_scene(*point))

        end = world_to_scene(*lane.centerline[-1])
        prev = world_to_scene(*lane.centerline[-2])
        angle = math.atan2(end.y() - prev.y(), end.x() - prev.x())
        self.path_item.set_arrow_path(path, angle)
        mid_point = lane.centerline[len(lane.centerline) // 2]
        self.label_item.setPos(world_to_scene(*mid_point) + QtCore.QPointF(8.0, -18.0))

    def rebuild_successors(
        self,
        lane: LaneData,
        lane_items: dict[str, "LaneItem"],
        scene: QtWidgets.QGraphicsScene,
    ) -> None:
        for successor_item in self.successor_items:
            if successor_item.scene():
                scene.removeItem(successor_item)
        self.successor_items.clear()

        for successor_id in lane.successors:
            target_item = lane_items.get(successor_id)
            if target_item is None:
                continue
            successor_item = ArrowPathItem(
                QtGui.QPen(QtGui.QColor("#d97706"), 1.6, QtCore.Qt.DashLine)
            )
            successor_item.setZValue(2)
            start = self.path_item.path().pointAtPercent(1.0)
            end = target_item.path_item.path().pointAtPercent(0.0)
            control = QtCore.QPointF(
                (start.x() + end.x()) / 2.0,
                (start.y() + end.y()) / 2.0 - 24.0,
            )
            path = QtGui.QPainterPath(start)
            path.quadTo(control, end)
            tangent = end - control
            angle = math.atan2(tangent.y(), tangent.x())
            successor_item.set_arrow_path(path, angle)
            self.successor_items.append(successor_item)
            scene.addItem(successor_item)

    def apply_selection_style(self, selected: bool) -> None:
        color = QtGui.QColor("#2563eb") if selected else QtGui.QColor("#1f2933")
        self.path_item.setPen(QtGui.QPen(color, 3.0 if selected else 2.4))
        self.path_item.arrow_head.setBrush(color)
        self.path_item.arrow_head.setPen(QtGui.QPen(color))
        self.label_item.setVisible(selected)
        for point_item in self.point_items:
            point_item.setBrush(
                QtGui.QBrush(QtGui.QColor("#bfdbfe") if selected else QtGui.QColor("#ffffff"))
            )


class LaneScene(QtWidgets.QGraphicsScene):
    def __init__(self, controller: "EditorController") -> None:
        super().__init__()
        self.controller = controller
        self.setBackgroundBrush(QtGui.QColor("#f8fafc"))
        self.grid_pen = QtGui.QPen(QtGui.QColor("#e2e8f0"))
        self.axis_pen = QtGui.QPen(QtGui.QColor("#94a3b8"))

    def drawBackground(self, painter: QtGui.QPainter, rect: QtCore.QRectF) -> None:
        super().drawBackground(painter, rect)
        grid = 0.1 * SCENE_SCALE
        left = math.floor(rect.left() / grid) * grid
        top = math.floor(rect.top() / grid) * grid

        painter.setPen(self.grid_pen)
        x = left
        while x <= rect.right():
            painter.drawLine(QtCore.QLineF(x, rect.top(), x, rect.bottom()))
            x += grid

        y = top
        while y <= rect.bottom():
            painter.drawLine(QtCore.QLineF(rect.left(), y, rect.right(), y))
            y += grid

        painter.setPen(self.axis_pen)
        painter.drawLine(QtCore.QLineF(0.0, rect.top(), 0.0, rect.bottom()))
        painter.drawLine(QtCore.QLineF(rect.left(), 0.0, rect.right(), 0.0))

    def mousePressEvent(self, event: QtWidgets.QGraphicsSceneMouseEvent) -> None:
        if self.controller.model.session.mode == "create":
            self.controller.add_create_point(event.scenePos())
            event.accept()
            return
        if self.controller.model.session.mode == "successor":
            self.controller.handle_successor_click(self.itemAt(event.scenePos(), QtGui.QTransform()))
            event.accept()
            return
        super().mousePressEvent(event)
        if not self.selectedItems():
            self.controller.clear_selection()


class GraphicsView(QtWidgets.QGraphicsView):
    def __init__(self, scene: LaneScene) -> None:
        super().__init__(scene)
        self.setRenderHints(QtGui.QPainter.Antialiasing | QtGui.QPainter.TextAntialiasing)
        self.setViewportUpdateMode(QtWidgets.QGraphicsView.BoundingRectViewportUpdate)
        self.setTransformationAnchor(QtWidgets.QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QtWidgets.QGraphicsView.AnchorUnderMouse)
        self.setFrameShape(QtWidgets.QFrame.NoFrame)
        self._panning = False
        self._pan_start = QtCore.QPoint()

    def wheelEvent(self, event: QtGui.QWheelEvent) -> None:
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        self.scale(factor, factor)

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:
        is_background = self.itemAt(event.pos()) is None
        should_pan = (
            event.button() == QtCore.Qt.LeftButton and is_background
        ) or event.button() == QtCore.Qt.MiddleButton
        if should_pan:
            self._panning = True
            self._pan_start = event.pos()
            self.setCursor(QtCore.Qt.ClosedHandCursor)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QtGui.QMouseEvent) -> None:
        if self._panning:
            delta = event.pos() - self._pan_start
            self._pan_start = event.pos()
            self.horizontalScrollBar().setValue(self.horizontalScrollBar().value() - delta.x())
            self.verticalScrollBar().setValue(self.verticalScrollBar().value() - delta.y())
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QtGui.QMouseEvent) -> None:
        if self._panning and event.button() in {QtCore.Qt.LeftButton, QtCore.Qt.MiddleButton}:
            self._panning = False
            self.setCursor(QtCore.Qt.ArrowCursor)
            event.accept()
            return
        super().mouseReleaseEvent(event)


class TransformDialog(QtWidgets.QDialog):
    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Transform Graph")
        layout = QtWidgets.QFormLayout(self)

        self.scale_x = QtWidgets.QDoubleSpinBox()
        self.scale_x.setDecimals(6)
        self.scale_x.setRange(-1000.0, 1000.0)
        self.scale_x.setValue(1.0)
        self.scale_y = QtWidgets.QDoubleSpinBox()
        self.scale_y.setDecimals(6)
        self.scale_y.setRange(-1000.0, 1000.0)
        self.scale_y.setValue(1.0)
        self.origin_x = QtWidgets.QDoubleSpinBox()
        self.origin_x.setDecimals(6)
        self.origin_x.setRange(-1000.0, 1000.0)
        self.origin_y = QtWidgets.QDoubleSpinBox()
        self.origin_y.setDecimals(6)
        self.origin_y.setRange(-1000.0, 1000.0)

        layout.addRow("Scale X", self.scale_x)
        layout.addRow("Scale Y", self.scale_y)
        layout.addRow("Origin X", self.origin_x)
        layout.addRow("Origin Y", self.origin_y)

        buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addRow(buttons)

    def values(self) -> tuple[float, float, float, float]:
        return (
            self.scale_x.value(),
            self.scale_y.value(),
            self.origin_x.value(),
            self.origin_y.value(),
        )


class EditorWindow(QtWidgets.QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.resize(1400, 900)
        self.setStyleSheet(
            """
            QMainWindow { background: #f8fafc; }
            QToolBar { spacing: 6px; padding: 6px; border: 0; background: #ffffff; }
            QDockWidget::title { background: #e2e8f0; padding: 8px; }
            QGroupBox { font-weight: 600; border: 1px solid #cbd5e1; border-radius: 8px; margin-top: 12px; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
            QPushButton { padding: 6px 10px; }
            QListWidget, QLineEdit, QDoubleSpinBox, QComboBox { min-height: 28px; }
            """
        )
        self._building_panel = False
        self.controller: EditorController | None = None
        self.scene: LaneScene | None = None
        self.view: GraphicsView | None = None
        self.lane_items: dict[str, LaneItem] = {}
        self.pending_preview: QtWidgets.QGraphicsPathItem | None = None
        self.transform_dialog_factory = lambda: TransformDialog(self)
        self._build_toolbar()
        self._build_dock()

    def attach_controller(self, controller: "EditorController") -> None:
        self.controller = controller
        self.scene = LaneScene(controller)
        self.scene.selectionChanged.connect(controller.handle_scene_selection_changed)
        self.view = GraphicsView(self.scene)
        self.setCentralWidget(self.view)

    def _build_toolbar(self) -> None:
        toolbar = self.addToolBar("Tools")
        toolbar.setMovable(False)
        self.select_action = QtWidgets.QAction("Select", self, checkable=True)
        self.create_action = QtWidgets.QAction("New Lane", self, checkable=True)
        self.successor_action = QtWidgets.QAction("Connect Successor", self, checkable=True)
        self.finish_action = QtWidgets.QAction("Finish Lane", self)
        self.cancel_action = QtWidgets.QAction("Cancel Draft", self)
        self.delete_action = QtWidgets.QAction("Delete Lane", self)
        self.transform_action = QtWidgets.QAction("Transform", self)
        self.fit_action = QtWidgets.QAction("Fit View", self)
        self.save_action = QtWidgets.QAction("Save", self)
        self.save_action.setShortcut(QtGui.QKeySequence.Save)

        self.mode_group = QtWidgets.QActionGroup(self)
        self.mode_group.setExclusive(True)
        for action in [self.select_action, self.create_action, self.successor_action]:
            self.mode_group.addAction(action)
            toolbar.addAction(action)
        toolbar.addSeparator()
        for action in [
            self.finish_action,
            self.cancel_action,
            self.delete_action,
            self.transform_action,
            self.fit_action,
            self.save_action,
        ]:
            toolbar.addAction(action)

    def _build_dock(self) -> None:
        dock = QtWidgets.QDockWidget("Inspector", self)
        dock.setFeatures(QtWidgets.QDockWidget.NoDockWidgetFeatures)
        panel = uic.loadUi(str(INSPECTOR_UI_PATH))

        self.mode_badge = panel.findChild(QtWidgets.QLabel, "mode_badge")
        self.selection_summary = panel.findChild(QtWidgets.QLabel, "selection_summary")
        self.lane_id_edit = panel.findChild(QtWidgets.QLineEdit, "lane_id_edit")
        self.kind_combo = panel.findChild(QtWidgets.QComboBox, "kind_combo")
        self.width_spin = panel.findChild(QtWidgets.QDoubleSpinBox, "width_spin")
        self.successors_list = panel.findChild(QtWidgets.QListWidget, "successors_list")
        self.remove_successor_btn = panel.findChild(QtWidgets.QPushButton, "remove_successor_btn")
        self.point_index_label = panel.findChild(QtWidgets.QLabel, "point_index_label")
        self.point_x_spin = panel.findChild(QtWidgets.QDoubleSpinBox, "point_x_spin")
        self.point_y_spin = panel.findChild(QtWidgets.QDoubleSpinBox, "point_y_spin")
        self.draft_summary = panel.findChild(QtWidgets.QLabel, "draft_summary")

        required_widgets = {
            "mode_badge": self.mode_badge,
            "selection_summary": self.selection_summary,
            "lane_id_edit": self.lane_id_edit,
            "kind_combo": self.kind_combo,
            "width_spin": self.width_spin,
            "successors_list": self.successors_list,
            "remove_successor_btn": self.remove_successor_btn,
            "point_index_label": self.point_index_label,
            "point_x_spin": self.point_x_spin,
            "point_y_spin": self.point_y_spin,
            "draft_summary": self.draft_summary,
        }
        missing = [name for name, widget in required_widgets.items() if widget is None]
        if missing:
            raise RuntimeError(f"Inspector UI is missing required widgets: {', '.join(missing)}")

        self.kind_combo.clear()
        self.kind_combo.addItems(LANE_KIND_OPTIONS)
        dock.setWidget(panel)
        self.addDockWidget(QtCore.Qt.RightDockWidgetArea, dock)

    def bind_controller(self, controller: "EditorController") -> None:
        self.select_action.triggered.connect(lambda: controller.set_mode("select"))
        self.create_action.triggered.connect(lambda: controller.set_mode("create"))
        self.successor_action.triggered.connect(lambda: controller.set_mode("successor"))
        self.finish_action.triggered.connect(controller.finish_pending_lane)
        self.cancel_action.triggered.connect(controller.cancel_active_mode)
        self.delete_action.triggered.connect(controller.delete_selected_lane)
        self.transform_action.triggered.connect(controller.request_transform)
        self.fit_action.triggered.connect(self.fit_scene)
        self.save_action.triggered.connect(controller.save)
        self.lane_id_edit.editingFinished.connect(controller.apply_lane_property_changes)
        self.kind_combo.currentTextChanged.connect(lambda _text: controller.apply_lane_property_changes())
        self.width_spin.valueChanged.connect(lambda _value: controller.apply_lane_property_changes())
        self.point_x_spin.valueChanged.connect(lambda _value: controller.apply_point_property_changes())
        self.point_y_spin.valueChanged.connect(lambda _value: controller.apply_point_property_changes())
        self.remove_successor_btn.clicked.connect(controller.remove_selected_successor)

    def lane_item_from_graphics(self, item: QtWidgets.QGraphicsItem | None) -> LaneItem | None:
        if isinstance(item, LanePointItem):
            return self.lane_items.get(item.lane_id)
        current = item
        while current is not None:
            lane_id = current.data(0)
            if lane_id:
                return self.lane_items.get(str(lane_id))
            current = current.parentItem()
        return None

    def render_model(self, model: EditorModel) -> None:
        if self.scene is None or self.view is None:
            return

        current_ids = {lane.lane_id for lane in model.document.lanes}
        existing_ids = set(self.lane_items)

        for lane_id in existing_ids - current_ids:
            lane_item = self.lane_items.pop(lane_id)
            lane_item.remove_from_scene(self.scene)

        for lane in model.document.lanes:
            lane_item = self.lane_items.get(lane.lane_id)
            if lane_item is None:
                lane_item = LaneItem(lane.lane_id, self.controller)
                self.lane_items[lane.lane_id] = lane_item
                lane_item.add_to_scene(self.scene)
            lane_item.sync_from_lane(lane, self.scene)

        for lane in model.document.lanes:
            lane_item = self.lane_items[lane.lane_id]
            lane_item.rebuild_successors(lane, self.lane_items, self.scene)
            lane_item.apply_selection_style(model.session.selected_lane_id == lane.lane_id)

        self.render_pending_preview(model.session.pending_lane_points)
        self.refresh_inspector(model)
        self.update_mode_banner(model)
        self.update_window_title(model)

    def render_pending_preview(self, pending_points: list[tuple[float, float]]) -> None:
        if self.scene is None:
            return
        if not pending_points:
            if self.pending_preview is not None and self.pending_preview.scene():
                self.scene.removeItem(self.pending_preview)
            self.pending_preview = None
            return

        if self.pending_preview is None:
            self.pending_preview = QtWidgets.QGraphicsPathItem()
            self.pending_preview.setPen(QtGui.QPen(QtGui.QColor("#16a34a"), 2.0, QtCore.Qt.DashLine))
            self.pending_preview.setZValue(40)
            self.scene.addItem(self.pending_preview)

        path = QtGui.QPainterPath()
        path.moveTo(world_to_scene(*pending_points[0]))
        for point in pending_points[1:]:
            path.lineTo(world_to_scene(*point))
        self.pending_preview.setPath(path)

    def refresh_inspector(self, model: EditorModel) -> None:
        self._building_panel = True
        lane = model.selected_lane()
        selected_point = model.selected_point()

        if lane is None:
            self.lane_id_edit.setText("")
            self.kind_combo.setCurrentText(LANE_KIND_OPTIONS[0])
            self.width_spin.setValue(model.document.default_lane_width)
            self.successors_list.clear()
            self.point_index_label.setText("-")
            self.point_x_spin.setValue(0.0)
            self.point_y_spin.setValue(0.0)
            self.selection_summary.setText("No lane selected")
        else:
            self.lane_id_edit.setText(lane.lane_id)
            self.kind_combo.setCurrentText(lane.kind)
            self.width_spin.setValue(lane.width)
            self.successors_list.clear()
            self.successors_list.addItems(lane.successors)
            summary = f"Lane: {lane.lane_id}\nPoints: {len(lane.centerline)}  Successors: {len(lane.successors)}"
            if selected_point is not None:
                point_index, (x, y) = selected_point
                self.point_index_label.setText(str(point_index))
                self.point_x_spin.setValue(x)
                self.point_y_spin.setValue(y)
                summary += f"\nPoint {point_index}: ({format_coord(x)}, {format_coord(y)})"
            else:
                self.point_index_label.setText("-")
                self.point_x_spin.setValue(0.0)
                self.point_y_spin.setValue(0.0)
            self.selection_summary.setText(summary)

        lane_enabled = lane is not None
        point_enabled = selected_point is not None
        for widget in [
            self.lane_id_edit,
            self.kind_combo,
            self.width_spin,
            self.successors_list,
            self.remove_successor_btn,
        ]:
            widget.setEnabled(lane_enabled)
        for widget in [self.point_x_spin, self.point_y_spin]:
            widget.setEnabled(point_enabled)
        self._building_panel = False

    def update_mode_banner(self, model: EditorModel) -> None:
        mode_text = {
            "select": "Mode: Select",
            "create": "Mode: Create Lane",
            "successor": "Mode: Connect Successor",
        }[model.session.mode]
        details: list[str] = []
        if model.session.mode == "create":
            details.append(f"{len(model.session.pending_lane_points)} draft points")
        if model.session.mode == "successor" and model.session.pending_successor_source:
            details.append(f"source={model.session.pending_successor_source}")
        if details:
            mode_text = f"{mode_text} | " + ", ".join(details)
        self.mode_badge.setText(mode_text)

        if model.session.mode == "create":
            if model.session.pending_lane_points:
                x, y = model.session.pending_lane_points[-1]
                self.draft_summary.setText(
                    f"Draft lane with {len(model.session.pending_lane_points)} points.\n"
                    f"Last point: ({format_coord(x)}, {format_coord(y)})"
                )
            else:
                self.draft_summary.setText("Click the canvas to place the first point.")
        elif model.session.mode == "successor":
            if model.session.pending_successor_source:
                self.draft_summary.setText(
                    f"Source lane selected: {model.session.pending_successor_source}\n"
                    "Click the target lane to finish the connection."
                )
            else:
                self.draft_summary.setText("Click a source lane, then click a target lane.")
        else:
            self.draft_summary.setText("No active draft")

        self.finish_action.setEnabled(model.session.mode == "create")
        self.cancel_action.setEnabled(model.session.mode in {"create", "successor"})
        self.select_action.setChecked(model.session.mode == "select")
        self.create_action.setChecked(model.session.mode == "create")
        self.successor_action.setChecked(model.session.mode == "successor")

    def update_window_title(self, model: EditorModel) -> None:
        marker = "*" if model.session.dirty else ""
        self.setWindowTitle(f"Lane Graph Editor{marker} - {model.document.path}")

    def fit_scene(self) -> None:
        if self.scene is None or self.view is None:
            return
        rect = self.scene.itemsBoundingRect()
        if rect.isNull():
            rect = QtCore.QRectF(-100.0, -100.0, 200.0, 200.0)
        rect.adjust(-80.0, -80.0, 80.0, 80.0)
        self.scene.setSceneRect(rect)
        self.view.fitInView(rect, QtCore.Qt.KeepAspectRatio)

    def show_error(self, message: str) -> None:
        QtWidgets.QMessageBox.warning(self, "Lane Graph Editor", message)

    def ask_transform(self) -> tuple[float, float, float, float] | None:
        dialog = self.transform_dialog_factory()
        if dialog.exec_() != QtWidgets.QDialog.Accepted:
            return None
        return dialog.values()

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        if self.controller is None or not self.controller.model.session.dirty:
            event.accept()
            return
        reply = QtWidgets.QMessageBox.question(
            self,
            "Unsaved Changes",
            "There are unsaved changes. Close anyway?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
        )
        if reply == QtWidgets.QMessageBox.Yes:
            event.accept()
        else:
            event.ignore()


class EditorController(QtCore.QObject):
    def __init__(self, model: EditorModel, window: EditorWindow) -> None:
        super().__init__()
        self.model = model
        self.window = window
        self.window.attach_controller(self)
        self.window.bind_controller(self)

        self.model.changed.connect(self.render)
        self.model.selection_changed.connect(self.render)
        self.model.mode_changed.connect(self.render)
        self.model.error_raised.connect(self.window.show_error)
        self.model.save_succeeded.connect(self.window.statusBar().showMessage)
        self.render()

    def render(self) -> None:
        self.window.render_model(self.model)

    def set_mode(self, mode: str) -> None:
        self.model.set_mode(mode)
        self.window.statusBar().showMessage(
            {
                "select": "Select mode: click a lane or point. Drag empty background to pan.",
                "create": "Create mode: click the canvas to add lane points.",
                "successor": "Successor mode: click source lane, then target lane.",
            }[mode]
        )

    def cancel_active_mode(self) -> None:
        self.model.cancel_active_mode()

    def clear_selection(self) -> None:
        self.model.select_lane(None)

    def select_lane(self, lane_id: str | None) -> None:
        self.model.select_lane(lane_id)

    def select_point(self, lane_id: str, point_index: int) -> None:
        self.model.select_point(lane_id, point_index)

    def move_point(self, lane_id: str, point_index: int, x: float, y: float) -> None:
        self.model.update_point(lane_id, point_index, x, y)

    def add_create_point(self, scene_pos: QtCore.QPointF) -> None:
        x, y = scene_to_world(scene_pos)
        self.model.add_create_point(x, y)
        self.window.statusBar().showMessage(
            f"New lane points: {len(self.model.session.pending_lane_points)}  last=({format_coord(x)}, {format_coord(y)})"
        )

    def finish_pending_lane(self) -> None:
        if self.model.finish_pending_lane():
            self.window.fit_scene()

    def handle_successor_click(self, item: QtWidgets.QGraphicsItem | None) -> None:
        lane_item = self.window.lane_item_from_graphics(item)
        if lane_item is None:
            return
        if self.model.session.pending_successor_source is None:
            self.model.start_successor(lane_item.lane_id)
            self.window.statusBar().showMessage(f"Successor source selected: {lane_item.lane_id}")
            return
        if self.model.connect_successor(lane_item.lane_id):
            self.window.statusBar().showMessage("Successor connected.")

    def remove_selected_successor(self) -> None:
        self.model.remove_selected_successor(self.window.successors_list.currentRow())

    def delete_selected_lane(self) -> None:
        if self.model.delete_selected_lane():
            self.window.statusBar().showMessage("Lane deleted.")

    def request_transform(self) -> None:
        values = self.window.ask_transform()
        if values is None:
            return
        self.model.apply_transform(*values)
        self.window.fit_scene()

    def apply_lane_property_changes(self) -> None:
        if self.window._building_panel:
            return
        lane = self.model.selected_lane()
        if lane is None:
            return
        if not self.model.update_lane_properties(
            lane.lane_id,
            self.window.lane_id_edit.text(),
            self.window.kind_combo.currentText(),
            self.window.width_spin.value(),
        ):
            self.render()

    def apply_point_property_changes(self) -> None:
        if self.window._building_panel:
            return
        selected_point = self.model.selected_point()
        lane = self.model.selected_lane()
        if selected_point is None or lane is None:
            return
        point_index, _point = selected_point
        self.model.update_point(
            lane.lane_id,
            point_index,
            self.window.point_x_spin.value(),
            self.window.point_y_spin.value(),
        )

    def handle_scene_selection_changed(self) -> None:
        if self.model.session.mode != "select" or self.window.scene is None:
            return
        selected_items = self.window.scene.selectedItems()
        if not selected_items:
            if self.model.session.selected_point_index is None:
                self.model.select_lane(None)
            return
        selected_item = selected_items[-1]
        if isinstance(selected_item, LanePointItem):
            self.model.select_point(selected_item.lane_id, selected_item.index)
            return
        lane_item = self.window.lane_item_from_graphics(selected_item)
        self.model.select_lane(lane_item.lane_id if lane_item else None)

    def save(self) -> None:
        self.model.save()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Edit araseo_navigation lane_graph.yaml")
    parser.add_argument(
        "--graph",
        default=str(PACKAGE_ROOT / "params" / "lane_graph.yaml"),
        help="Path to lane_graph.yaml",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    app = QtWidgets.QApplication(sys.argv)
    document = LaneDocument.load(Path(args.graph).expanduser().resolve())
    model = EditorModel(document)
    window = EditorWindow()
    controller = EditorController(model, window)
    window.controller = controller
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
