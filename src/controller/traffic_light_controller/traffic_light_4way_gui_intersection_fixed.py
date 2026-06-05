
import sys
import socket
import time
from dataclasses import dataclass
from PyQt6.QtCore import Qt, QTimer, QRectF
from PyQt6.QtGui import QColor, QPainter, QPen, QBrush, QFont
from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
    QGridLayout, QGroupBox, QLineEdit, QSpinBox, QMessageBox, QTabWidget,
    QComboBox, QFormLayout, QFrame, QTextEdit, QSizePolicy
)

TCP_PORT = 5000
SOCKET_TIMEOUT = 0.2


class SmallSignalWidget(QWidget):
    def __init__(self, title="Signal"):
        super().__init__()
        self.title = title
        self.current = "R"
        self.blink = False
        self.setMinimumSize(110, 250)
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)

    def set_state(self, state: str, blink: bool = False):
        self.current = state
        self.blink = blink
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor(245, 247, 250))
        p.drawRoundedRect(self.rect().adjusted(2, 2, -2, -2), 18, 18)

        p.setPen(QPen(QColor(60, 60, 60), 1))
        p.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        p.drawText(0, 10, self.width(), 24, Qt.AlignmentFlag.AlignCenter, self.title)

        body = QRectF(self.width()/2 - 28, 42, 56, 150)
        p.setPen(QPen(QColor(20, 20, 20), 2))
        p.setBrush(QColor(55, 58, 64))
        p.drawRoundedRect(body, 18, 18)

        positions = [("R", 70), ("Y", 118), ("G", 166)]
        on_map = {"R": QColor(225, 55, 55), "Y": QColor(235, 205, 45), "G": QColor(35, 190, 90)}
        off = QColor(90, 95, 100)

        for key, y in positions:
            color = on_map[key] if self.current == key else off
            p.setBrush(QBrush(color))
            p.setPen(QPen(QColor(15, 15, 15), 2))
            p.drawEllipse(int(self.width()/2 - 15), y - 15, 30, 30)

        p.setPen(QPen(QColor(90, 90, 90), 1))
        p.setFont(QFont("Arial", 9))
        blink_txt = " / BLINK" if self.blink else ""
        p.drawText(0, 205, self.width(), 20, Qt.AlignmentFlag.AlignCenter, f"{self.current}{blink_txt}")


class IntersectionWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.states = {
            1: {"lamp": "R", "blink": False},
            2: {"lamp": "R", "blink": False},
            3: {"lamp": "R", "blink": False},
            4: {"lamp": "R", "blink": False},
        }
        self.setMinimumSize(650, 500)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def set_signal_state(self, idx: int, lamp: str, blink: bool = False):
        self.states[idx] = {"lamp": lamp, "blink": blink}
        self.update()

    def _lamp_color(self, key: str, active_key: str):
        on = {"R": QColor(225, 55, 55), "Y": QColor(235, 205, 45), "G": QColor(35, 190, 90)}
        off = QColor(80, 85, 90)
        return on[key] if key == active_key else off

    def _draw_signal_head(self, p: QPainter, cx: int, cy: int, active: str, label: str, direction: str):
        p.setPen(QPen(QColor(25, 25, 25), 2))
        p.setBrush(QColor(50, 53, 58))
        p.drawRoundedRect(cx - 24, cy - 52, 48, 104, 14, 14)

        order = [("R", -28), ("Y", 0), ("G", 28)]
        for key, dy in order:
            p.setBrush(self._lamp_color(key, active))
            p.setPen(QPen(QColor(10, 10, 10), 2))
            p.drawEllipse(cx - 11, cy + dy - 11, 22, 22)

        p.setPen(QPen(QColor(35, 35, 35), 1))
        p.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        p.drawText(cx - 55, cy + 66, 110, 18, Qt.AlignmentFlag.AlignCenter, label)

        p.setFont(QFont("Arial", 9))
        p.drawText(cx - 55, cy + 84, 110, 18, Qt.AlignmentFlag.AlignCenter, direction)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        p.fillRect(self.rect(), QColor(238, 241, 245))

        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor(205, 224, 203))
        p.drawRect(0, 0, self.width(), self.height())

        road_w = 150
        cx = self.width() // 2
        cy = self.height() // 2

        p.setBrush(QColor(55, 58, 64))
        p.drawRect(cx - road_w // 2, 0, road_w, self.height())
        p.drawRect(0, cy - road_w // 2, self.width(), road_w)

        p.setPen(QPen(QColor(245, 245, 245), 3, Qt.PenStyle.DashLine))
        p.drawLine(cx, 0, cx, cy - road_w // 2 - 18)
        p.drawLine(cx, cy + road_w // 2 + 18, cx, self.height())
        p.drawLine(0, cy, cx - road_w // 2 - 18, cy)
        p.drawLine(cx + road_w // 2 + 18, cy, self.width(), cy)

        p.setPen(QPen(QColor(255, 255, 255), 5))
        p.drawLine(cx - 55, cy - road_w // 2 + 22, cx + 55, cy - road_w // 2 + 22)
        p.drawLine(cx - 55, cy + road_w // 2 - 22, cx + 55, cy + road_w // 2 - 22)
        p.drawLine(cx - road_w // 2 + 22, cy - 55, cx - road_w // 2 + 22, cy + 55)
        p.drawLine(cx + road_w // 2 - 22, cy - 55, cx + road_w // 2 - 22, cy + 55)

        p.setPen(QPen(QColor(235, 235, 235), 2))
        p.setBrush(QColor(70, 74, 80))
        p.drawRoundedRect(cx - 48, cy - 48, 96, 96, 10, 10)

        p.setPen(QPen(QColor(40, 40, 40), 1))
        p.setFont(QFont("Arial", 13, QFont.Weight.Bold))
        p.drawText(0, 10, self.width(), 28, Qt.AlignmentFlag.AlignCenter, "교차로 통합 모니터")

        self._draw_signal_head(p, cx, cy - 118, self.states[1]["lamp"], "신호등 1", "북쪽 진입")
        self._draw_signal_head(p, cx + 118, cy, self.states[2]["lamp"], "신호등 2", "동쪽 진입")
        self._draw_signal_head(p, cx, cy + 118, self.states[3]["lamp"], "신호등 3", "남쪽 진입")
        self._draw_signal_head(p, cx - 118, cy, self.states[4]["lamp"], "신호등 4", "서쪽 진입")

        greens = [str(i) for i, v in self.states.items() if v["lamp"] == "G"]
        summary = "현재 초록: " + (", ".join(greens) if greens else "없음")
        p.setFont(QFont("Arial", 11, QFont.Weight.Bold))
        p.setPen(QPen(QColor(35, 35, 35), 1))
        p.drawText(0, self.height() - 34, self.width(), 22, Qt.AlignmentFlag.AlignCenter, summary)


@dataclass
class NodeState:
    node_id: int
    ip: str = ""
    sock: socket.socket | None = None
    connected: bool = False
    lamp: str = "R"
    blink: bool = False
    last_rx: float = 0.0
    buffer: str = ""


class NodeClient:
    def __init__(self, node_id: int):
        self.state = NodeState(node_id=node_id)

    def connect(self, ip: str):
        self.disconnect()
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(2.0)
        s.connect((ip, TCP_PORT))
        s.settimeout(SOCKET_TIMEOUT)
        self.state.ip = ip
        self.state.sock = s
        self.state.connected = True
        self.state.last_rx = time.time()

    def disconnect(self):
        if self.state.sock:
            try:
                self.state.sock.close()
            except Exception:
                pass
        self.state.sock = None
        self.state.connected = False

    def send(self, line: str):
        if not self.state.connected or not self.state.sock:
            raise ConnectionError(f"Signal {self.state.node_id} not connected")
        self.state.sock.sendall((line + "\n").encode("utf-8"))

    def poll_lines(self):
        lines = []
        if not self.state.connected or not self.state.sock:
            return lines
        try:
            while True:
                data = self.state.sock.recv(4096)
                if not data:
                    raise ConnectionError("socket closed")
                self.state.buffer += data.decode("utf-8", errors="ignore")
                if "\n" not in self.state.buffer:
                    break
                parts = self.state.buffer.split("\n")
                self.state.buffer = parts[-1]
                for p in parts[:-1]:
                    line = p.strip()
                    if line:
                        self.state.last_rx = time.time()
                        lines.append(line)
        except TimeoutError:
            pass
        except socket.timeout:
            pass
        except BlockingIOError:
            pass
        except Exception:
            self.disconnect()
        return lines


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("4개 신호등 교차로 관제")
        self.resize(1600, 980)

        self.nodes = [NodeClient(i) for i in range(1, 5)]

        self.system_mode = "MANUAL"
        self.current_green = 1
        self.auto_green_ms = 5000
        self.auto_yellow_ms = 2000
        self.auto_running = False
        self.auto_phase = "GREEN"
        self.phase_started = time.time()

        self.transition_active = False
        self.transition_from = None
        self.transition_to = None
        self.transition_started = 0.0
        self.transition_yellow_ms = 2000

        self.build_ui()
        self.setup_timers()

    def build_ui(self):
        root = QHBoxLayout(self)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(14)

        left_wrap = QVBoxLayout()
        left_wrap.setSpacing(12)

        top_group = QGroupBox("ESP32 연결")
        top_layout = QGridLayout(top_group)

        self.ip_edits = []
        self.conn_labels = []

        for i in range(4):
            top_layout.addWidget(QLabel(f"신호등 {i+1} IP"), i, 0)

            edit = QLineEdit()
            edit.setPlaceholderText(f"192.168.0.10{i+1}")
            self.ip_edits.append(edit)
            top_layout.addWidget(edit, i, 1)

            btn_conn = QPushButton("연결")
            btn_disc = QPushButton("해제")
            btn_stat = QPushButton("상태")
            btn_conn.clicked.connect(lambda _, idx=i: self.connect_node(idx))
            btn_disc.clicked.connect(lambda _, idx=i: self.disconnect_node(idx))
            btn_stat.clicked.connect(lambda _, idx=i: self.request_status(idx))

            top_layout.addWidget(btn_conn, i, 2)
            top_layout.addWidget(btn_disc, i, 3)
            top_layout.addWidget(btn_stat, i, 4)

            conn = QLabel("미연결")
            conn.setStyleSheet("font-weight:bold;color:#d93025;")
            self.conn_labels.append(conn)
            top_layout.addWidget(conn, i, 5)

        left_wrap.addWidget(top_group)

        status_group = QGroupBox("시스템 상태")
        status_layout = QVBoxLayout(status_group)
        self.system_mode_label = QLabel("시스템 모드: MANUAL")
        self.active_green_label = QLabel("현재 초록 신호등: 없음")
        self.phase_label = QLabel("현재 단계: -")
        for lab in [self.system_mode_label, self.active_green_label, self.phase_label]:
            lab.setStyleSheet("font-size:15px;font-weight:bold;color:#1f2328;")
            status_layout.addWidget(lab)
        left_wrap.addWidget(status_group)

        self.tabs = QTabWidget()
        self.auto_tab = QWidget()
        self.manual_tab = QWidget()
        self.tabs.addTab(self.auto_tab, "자동")
        self.tabs.addTab(self.manual_tab, "수동/반자동")
        self.tabs.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        left_wrap.addWidget(self.tabs, 0)

        self.build_auto_tab()
        self.build_manual_tab()

        log_group = QGroupBox("로그")
        log_group.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)
        log_layout = QVBoxLayout(log_group)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMinimumHeight(260)
        self.log_text.setStyleSheet(
            "background:#f8f9fb;color:#1f2328;border:1px solid #d0d7de;border-radius:10px;padding:8px;"
        )
        log_layout.addWidget(self.log_text)
        left_wrap.addWidget(log_group, 1)

        left_frame = QFrame()
        left_frame.setLayout(left_wrap)
        left_frame.setMaximumWidth(620)
        left_frame.setMinimumWidth(520)
        root.addWidget(left_frame, 0)

        right_wrap = QVBoxLayout()
        right_wrap.setSpacing(12)

        cross_group = QGroupBox("교차로 화면")
        cross_layout = QVBoxLayout(cross_group)
        self.intersection_widget = IntersectionWidget()
        cross_layout.addWidget(self.intersection_widget)
        right_wrap.addWidget(cross_group, 3)

        card_group = QGroupBox("개별 신호등 상태")
        card_layout = QHBoxLayout(card_group)
        self.small_widgets = []
        self.card_labels = []
        for i in range(4):
            col = QVBoxLayout()
            w = SmallSignalWidget(f"Signal {i+1}")
            s = QLabel("STATE: R / BLINK: OFF")
            s.setAlignment(Qt.AlignmentFlag.AlignCenter)
            s.setStyleSheet("font-weight:bold;color:#1f2328;")
            self.small_widgets.append(w)
            self.card_labels.append(s)
            col.addWidget(w)
            col.addWidget(s)
            card_layout.addLayout(col)
        right_wrap.addWidget(card_group, 2)

        right_frame = QFrame()
        right_frame.setLayout(right_wrap)
        root.addWidget(right_frame, 1)

        self.setStyleSheet("""
            QWidget {
                background: #edf1f5;
                font-size: 13px;
                color: #1f2328;
            }
            QGroupBox {
                font-weight: bold;
                color: #1f2328;
                border: 1px solid #cfd6df;
                border-radius: 14px;
                margin-top: 8px;
                padding-top: 12px;
                background: #ffffff;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 6px 0 6px;
                color: #1f2328;
            }
            QLabel {
                color: #1f2328;
                background: transparent;
            }
            QPushButton {
                background: #1f6feb;
                color: white;
                border: none;
                border-radius: 10px;
                padding: 8px 12px;
                min-height: 18px;
                font-weight: bold;
            }
            QPushButton:hover {
                background: #1859bd;
            }
            QLineEdit, QSpinBox, QComboBox, QTextEdit {
                background: white;
                color: #1f2328;
                border: 1px solid #c4cbd5;
                border-radius: 8px;
                padding: 6px;
                min-height: 20px;
                selection-background-color: #1f6feb;
                selection-color: white;
            }
            QTabWidget::pane {
                border: 1px solid #d9dfe7;
                background: #ffffff;
                border-radius: 12px;
            }
            QTabBar::tab {
                background: #dfe6ef;
                color: #1f2328;
                padding: 8px 16px;
                border-top-left-radius: 8px;
                border-top-right-radius: 8px;
                margin-right: 4px;
                font-weight: bold;
            }
            QTabBar::tab:selected {
                background: #ffffff;
                color: #1f2328;
            }
        """)

        self.update_global_labels()

    def build_auto_tab(self):
        layout = QVBoxLayout(self.auto_tab)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        box = QGroupBox("자동 사이클 설정")
        form = QFormLayout(box)

        self.green_spin = QSpinBox()
        self.green_spin.setRange(1, 999)
        self.green_spin.setValue(5)
        self.green_spin.setSuffix(" 초")

        self.yellow_spin = QSpinBox()
        self.yellow_spin.setRange(1, 999)
        self.yellow_spin.setValue(2)
        self.yellow_spin.setSuffix(" 초")

        form.addRow("초록 유지", self.green_spin)
        form.addRow("노랑 유지", self.yellow_spin)

        row = QHBoxLayout()
        self.btn_auto_apply = QPushButton("시간 적용")
        self.btn_auto_start = QPushButton("자동 시작")
        self.btn_auto_stop = QPushButton("자동 중지")
        self.btn_force_init = QPushButton("전체 R 초기화")
        row.addWidget(self.btn_auto_apply)
        row.addWidget(self.btn_auto_start)
        row.addWidget(self.btn_auto_stop)
        row.addWidget(self.btn_force_init)
        form.addRow(row)

        info = QLabel(
            "자동 순서: 1 → 2 → 3 → 4 → 1\n"
            "전환 시 현재 초록 신호등은 먼저 노랑으로 바뀐 뒤 빨강이 되고,\n"
            "그 다음 신호등이 초록으로 바뀝니다."
        )
        info.setWordWrap(True)
        info.setStyleSheet("color:#1f2328;")
        layout.addWidget(box)
        layout.addWidget(info)
        layout.addStretch(0)

        self.btn_auto_apply.clicked.connect(self.apply_auto_config)
        self.btn_auto_start.clicked.connect(self.start_auto_mode)
        self.btn_auto_stop.clicked.connect(self.stop_auto_mode)
        self.btn_force_init.clicked.connect(self.force_all_red)

    def build_manual_tab(self):
        layout = QVBoxLayout(self.manual_tab)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        manual_group = QGroupBox("개별 수동 제어")
        manual_group.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        grid = QGridLayout(manual_group)

        self.manual_target_combo = QComboBox()
        for i in range(1, 5):
            self.manual_target_combo.addItem(f"신호등 {i}", i)

        self.blink_interval_spin = QSpinBox()
        self.blink_interval_spin.setRange(100, 5000)
        self.blink_interval_spin.setValue(500)
        self.blink_interval_spin.setSuffix(" ms")

        grid.addWidget(QLabel("대상"), 0, 0)
        grid.addWidget(self.manual_target_combo, 0, 1)

        btn_r = QPushButton("R")
        btn_y = QPushButton("Y")
        btn_g = QPushButton("G")
        btn_off = QPushButton("OFF")
        btn_r.clicked.connect(lambda: self.manual_set_selected("R"))
        btn_y.clicked.connect(lambda: self.manual_set_selected("Y"))
        btn_g.clicked.connect(lambda: self.manual_set_selected("G"))
        btn_off.clicked.connect(lambda: self.manual_set_selected("OFF"))

        grid.addWidget(btn_r, 1, 0)
        grid.addWidget(btn_y, 1, 1)
        grid.addWidget(btn_g, 1, 2)
        grid.addWidget(btn_off, 1, 3)

        btn_br = QPushButton("R 점멸")
        btn_by = QPushButton("Y 점멸")
        btn_bs = QPushButton("점멸 정지")
        btn_br.clicked.connect(lambda: self.manual_blink_selected("R"))
        btn_by.clicked.connect(lambda: self.manual_blink_selected("Y"))
        btn_bs.clicked.connect(self.manual_stop_blink_selected)

        grid.addWidget(QLabel("점멸 주기"), 2, 0)
        grid.addWidget(self.blink_interval_spin, 2, 1)
        grid.addWidget(btn_br, 2, 2)
        grid.addWidget(btn_by, 2, 3)
        grid.addWidget(btn_bs, 2, 4)

        layout.addWidget(manual_group, 0)

        semi_group = QGroupBox("반자동 전환")
        semi_group.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        semi_layout = QGridLayout(semi_group)

        self.semi_target_combo = QComboBox()
        for i in range(1, 5):
            self.semi_target_combo.addItem(f"신호등 {i}", i)

        self.semi_yellow_spin = QSpinBox()
        self.semi_yellow_spin.setRange(100, 10000)
        self.semi_yellow_spin.setValue(2000)
        self.semi_yellow_spin.setSuffix(" ms")

        semi_layout.addWidget(QLabel("다음 초록 대상"), 0, 0)
        semi_layout.addWidget(self.semi_target_combo, 0, 1)
        semi_layout.addWidget(QLabel("노랑 유지"), 0, 2)
        semi_layout.addWidget(self.semi_yellow_spin, 0, 3)

        self.btn_semi_change = QPushButton("현재 초록 → 대상 초록")
        self.btn_set_target_green_now = QPushButton("즉시 대상 G")
        semi_layout.addWidget(self.btn_semi_change, 1, 0, 1, 2)
        semi_layout.addWidget(self.btn_set_target_green_now, 1, 2, 1, 2)

        self.btn_semi_change.clicked.connect(self.start_semi_auto_change)
        self.btn_set_target_green_now.clicked.connect(self.force_target_green_now)

        note = QLabel("반자동: 현재 초록불이 있으면 Y를 거친 뒤 R로 바꾸고, 목표 신호등을 G로 전환합니다.")
        note.setWordWrap(True)
        note.setStyleSheet("color:#1f2328;")

        layout.addWidget(semi_group, 0)
        layout.addWidget(note, 0)
        layout.addStretch(1)

    def setup_timers(self):
        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self.poll_nodes)
        self.poll_timer.start(100)

        self.ping_timer = QTimer(self)
        self.ping_timer.timeout.connect(self.ping_nodes)
        self.ping_timer.start(2000)

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.request_all_status)
        self.status_timer.start(1500)

        self.logic_timer = QTimer(self)
        self.logic_timer.timeout.connect(self.process_logic)
        self.logic_timer.start(100)

    def append_log(self, msg: str):
        self.log_text.append(msg)

    def connect_node(self, idx_zero):
        ip = self.ip_edits[idx_zero].text().strip()
        if not ip:
            QMessageBox.warning(self, "경고", f"신호등 {idx_zero+1} IP를 입력하세요.")
            return
        try:
            self.nodes[idx_zero].connect(ip)
            self.conn_labels[idx_zero].setText(f"연결됨 ({ip})")
            self.conn_labels[idx_zero].setStyleSheet("font-weight:bold;color:#188038;")
            self.append_log(f"[INFO] 신호등 {idx_zero+1} 연결 성공: {ip}")
            self.request_status(idx_zero)
        except Exception as e:
            self.conn_labels[idx_zero].setText("연결 실패")
            self.conn_labels[idx_zero].setStyleSheet("font-weight:bold;color:#d93025;")
            QMessageBox.critical(self, "연결 오류", f"신호등 {idx_zero+1}: {e}")

    def disconnect_node(self, idx_zero):
        self.nodes[idx_zero].disconnect()
        self.conn_labels[idx_zero].setText("미연결")
        self.conn_labels[idx_zero].setStyleSheet("font-weight:bold;color:#d93025;")
        self.append_log(f"[INFO] 신호등 {idx_zero+1} 연결 해제")

    def safe_send(self, node_id: int, cmd: str):
        nc = self.nodes[node_id - 1]
        try:
            nc.send(cmd)
            self.append_log(f"[TX][{node_id}] {cmd}")
        except Exception as e:
            self.append_log(f"[ERR][{node_id}] {e}")
            self.disconnect_node(node_id - 1)

    def request_status(self, idx_zero):
        self.safe_send(idx_zero + 1, "STATUS")

    def request_all_status(self):
        for i in range(4):
            if self.nodes[i].state.connected:
                self.request_status(i)

    def ping_nodes(self):
        for i in range(4):
            if self.nodes[i].state.connected:
                self.safe_send(i + 1, "PING")

    def poll_nodes(self):
        for i, nc in enumerate(self.nodes):
            lines = nc.poll_lines()
            for line in lines:
                self.append_log(f"[RX][{i+1}] {line}")
                self.parse_line(i + 1, line)

    def parse_line(self, node_id: int, line: str):
        st = self.nodes[node_id - 1].state

        if line.startswith("HELLO,"):
            return
        if line == "PONG":
            if st.connected:
                self.conn_labels[node_id - 1].setText(f"연결됨 ({st.ip})")
                self.conn_labels[node_id - 1].setStyleSheet("font-weight:bold;color:#188038;")
            return

        if line.startswith("STATUS,"):
            parts = line.split(",")
            data = {}
            for p in parts[1:]:
                if "=" in p:
                    k, v = p.split("=", 1)
                    data[k] = v

            lamp = data.get("LAMP", st.lamp)
            blink = data.get("BLINK", "OFF") == "ON"
            st.lamp = lamp
            st.blink = blink
            self.refresh_signal_widgets(node_id, lamp, blink)
            return

        if line.startswith("OK,SET,"):
            lamp = line.split(",")[-1].strip()
            st.lamp = lamp
            st.blink = False
            self.refresh_signal_widgets(node_id, lamp, False)
            return

        if line.startswith("OK,BLINK,START"):
            st.blink = True
            self.refresh_signal_widgets(node_id, st.lamp, True)
            return

        if line.startswith("OK,BLINK,OFF"):
            st.blink = False
            self.refresh_signal_widgets(node_id, "OFF", False)
            st.lamp = "OFF"
            return

    def refresh_signal_widgets(self, node_id: int, lamp: str, blink: bool):
        self.small_widgets[node_id - 1].set_state(lamp, blink)
        self.card_labels[node_id - 1].setText(f"STATE: {lamp} / BLINK: {'ON' if blink else 'OFF'}")
        self.intersection_widget.set_signal_state(node_id, lamp, blink)
        self.update_global_labels()

    def update_global_labels(self):
        self.system_mode_label.setText(f"시스템 모드: {self.system_mode}")
        greens = [n.state.node_id for n in self.nodes if n.state.lamp == "G"]
        if len(greens) == 1:
            self.active_green_label.setText(f"현재 초록 신호등: {greens[0]}")
        elif len(greens) > 1:
            self.active_green_label.setText(f"현재 초록 신호등: 복수 {greens}")
        else:
            self.active_green_label.setText("현재 초록 신호등: 없음")

        if self.system_mode == "AUTO":
            self.phase_label.setText(f"현재 단계: AUTO / {self.auto_phase} / 기준={self.current_green}번")
        elif self.transition_active:
            self.phase_label.setText(f"현재 단계: 반자동 / {self.transition_from}→{self.transition_to}")
        else:
            self.phase_label.setText("현재 단계: MANUAL")

    def all_connected_check(self):
        disconnected = [n.state.node_id for n in self.nodes if not n.state.connected]
        if disconnected:
            QMessageBox.warning(self, "경고", f"미연결 신호등: {disconnected}")
            return False
        return True

    def apply_auto_config(self):
        self.auto_green_ms = self.green_spin.value() * 1000
        self.auto_yellow_ms = self.yellow_spin.value() * 1000
        self.append_log(f"[INFO] 자동 설정 적용: G={self.auto_green_ms}ms, Y={self.auto_yellow_ms}ms")

    def force_all_red(self):
        self.auto_running = False
        self.transition_active = False
        self.system_mode = "MANUAL"
        for i in range(1, 5):
            self.safe_send(i, "BLINK OFF")
            self.safe_send(i, "SET R")
        self.current_green = 1
        self.update_global_labels()

    def start_auto_mode(self):
        if not self.all_connected_check():
            return
        self.apply_auto_config()
        self.system_mode = "AUTO"
        self.auto_running = True
        self.transition_active = False
        self.current_green = 1
        self.auto_phase = "GREEN"
        self.phase_started = time.time()

        for i in range(1, 5):
            self.safe_send(i, "BLINK OFF")
            self.safe_send(i, "SET R")
        self.safe_send(self.current_green, "SET G")
        self.update_global_labels()
        self.append_log("[INFO] 자동 모드 시작")

    def stop_auto_mode(self):
        self.auto_running = False
        self.system_mode = "MANUAL"
        self.update_global_labels()
        self.append_log("[INFO] 자동 모드 중지")

    def process_logic(self):
        now = time.time()

        if self.auto_running and self.system_mode == "AUTO":
            elapsed_ms = int((now - self.phase_started) * 1000)

            if self.auto_phase == "GREEN":
                if elapsed_ms >= self.auto_green_ms:
                    self.safe_send(self.current_green, "SET Y")
                    self.auto_phase = "YELLOW"
                    self.phase_started = now
                    self.update_global_labels()

            elif self.auto_phase == "YELLOW":
                if elapsed_ms >= self.auto_yellow_ms:
                    self.safe_send(self.current_green, "SET R")
                    self.current_green = 1 if self.current_green == 4 else self.current_green + 1
                    self.safe_send(self.current_green, "SET G")
                    self.auto_phase = "GREEN"
                    self.phase_started = now
                    self.update_global_labels()

        if self.transition_active:
            elapsed_ms = int((now - self.transition_started) * 1000)
            if elapsed_ms >= self.transition_yellow_ms:
                if self.transition_from:
                    self.safe_send(self.transition_from, "SET R")
                if self.transition_to:
                    self.safe_send(self.transition_to, "SET G")
                self.transition_active = False
                self.system_mode = "MANUAL"
                self.current_green = self.transition_to if self.transition_to else self.current_green
                self.update_global_labels()
                self.append_log(f"[INFO] 반자동 전환 완료: {self.transition_from} -> {self.transition_to}")

    def manual_set_selected(self, lamp: str):
        target = self.manual_target_combo.currentData()
        self.auto_running = False
        self.transition_active = False
        self.system_mode = "MANUAL"

        if lamp == "G":
            for i in range(1, 5):
                if i == target:
                    self.safe_send(i, "SET G")
                else:
                    self.safe_send(i, "BLINK OFF")
                    self.safe_send(i, "SET R")
            self.current_green = target
        else:
            self.safe_send(target, f"SET {lamp}")
        self.update_global_labels()

    def manual_blink_selected(self, lamp: str):
        target = self.manual_target_combo.currentData()
        interval = self.blink_interval_spin.value()
        self.auto_running = False
        self.transition_active = False
        self.system_mode = "MANUAL"
        self.safe_send(target, f"BLINK {lamp} {interval}")
        self.update_global_labels()

    def manual_stop_blink_selected(self):
        target = self.manual_target_combo.currentData()
        self.safe_send(target, "BLINK OFF")

    def detect_current_green(self):
        greens = [n.state.node_id for n in self.nodes if n.state.lamp == "G"]
        return greens[0] if greens else None

    def start_semi_auto_change(self):
        target = self.semi_target_combo.currentData()
        yellow_ms = self.semi_yellow_spin.value()

        self.auto_running = False
        self.system_mode = "MANUAL"

        current_green = self.detect_current_green()
        if current_green == target:
            QMessageBox.information(self, "안내", f"신호등 {target}가 이미 초록불입니다.")
            return

        for i in range(1, 5):
            if i not in [target, current_green]:
                self.safe_send(i, "BLINK OFF")
                self.safe_send(i, "SET R")

        if current_green is None:
            self.safe_send(target, "SET G")
            self.current_green = target
            self.update_global_labels()
            self.append_log(f"[INFO] 반자동: 현재 초록 없음 -> {target}번 초록")
            return

        self.transition_active = True
        self.transition_from = current_green
        self.transition_to = target
        self.transition_yellow_ms = yellow_ms
        self.transition_started = time.time()

        self.safe_send(current_green, "SET Y")
        self.update_global_labels()
        self.append_log(f"[INFO] 반자동 시작: {current_green}번 Y 후 {target}번 G")

    def force_target_green_now(self):
        target = self.semi_target_combo.currentData()
        self.auto_running = False
        self.transition_active = False
        self.system_mode = "MANUAL"
        for i in range(1, 5):
            self.safe_send(i, "BLINK OFF")
            self.safe_send(i, "SET G" if i == target else "SET R")
        self.current_green = target
        self.update_global_labels()
        self.append_log(f"[INFO] 강제 전환: {target}번 초록")

    def closeEvent(self, event):
        for nc in self.nodes:
            nc.disconnect()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())
