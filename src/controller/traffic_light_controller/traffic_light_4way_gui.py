
import sys
import socket
import time
from dataclasses import dataclass
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QColor, QPainter, QPen, QBrush
from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
    QGridLayout, QGroupBox, QLineEdit, QSpinBox, QMessageBox, QTabWidget,
    QFrame, QComboBox, QFormLayout
)

TCP_PORT = 5000
SOCKET_TIMEOUT = 0.2


class LampWidget(QWidget):
    def __init__(self, title="Signal"):
        super().__init__()
        self.title = title
        self.current = "R"
        self.setMinimumSize(120, 280)

    def set_state(self, state: str):
        self.current = state
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        rect = self.rect().adjusted(10, 10, -10, -10)

        # body
        painter.setPen(QPen(Qt.GlobalColor.black, 2))
        painter.setBrush(QBrush(QColor(50, 50, 50)))
        painter.drawRoundedRect(rect.adjusted(20, 30, -20, -10), 18, 18)

        painter.setPen(QPen(Qt.GlobalColor.black, 1))
        painter.drawText(0, 0, self.width(), 26, Qt.AlignmentFlag.AlignCenter, self.title)

        colors = {
            "R": QColor(220, 30, 30),
            "Y": QColor(230, 210, 30),
            "G": QColor(20, 180, 60),
            "OFF": QColor(80, 80, 80)
        }

        centers = [
            (self.width() // 2, 85, "R"),
            (self.width() // 2, 145, "Y"),
            (self.width() // 2, 205, "G"),
        ]
        radius = 22

        for x, y, lamp in centers:
            on = self.current == lamp
            color = colors[lamp] if on else QColor(70, 70, 70)
            painter.setBrush(QBrush(color))
            painter.setPen(QPen(Qt.GlobalColor.black, 2))
            painter.drawEllipse(x - radius, y - radius, radius * 2, radius * 2)

        if self.current == "OFF":
            painter.setBrush(QBrush(QColor(100, 100, 100)))
            painter.setPen(QPen(Qt.GlobalColor.black, 1))
            painter.drawText(0, 240, self.width(), 20, Qt.AlignmentFlag.AlignCenter, "OFF")
        else:
            painter.drawText(0, 240, self.width(), 20, Qt.AlignmentFlag.AlignCenter, f"STATE: {self.current}")


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
        self.setWindowTitle("4개 신호등 통합 관제")
        self.resize(1450, 900)

        self.nodes = [NodeClient(i) for i in range(1, 5)]

        # coordinated auto state
        self.system_mode = "MANUAL"
        self.current_green = 1
        self.auto_green_ms = 5000
        self.auto_yellow_ms = 2000
        self.auto_running = False
        self.auto_phase = "GREEN"  # GREEN or YELLOW
        self.phase_started = time.time()

        # semi-auto transition
        self.transition_active = False
        self.transition_from = None
        self.transition_to = None
        self.transition_started = 0.0
        self.transition_yellow_ms = 2000

        self.build_ui()
        self.setup_timers()

    def build_ui(self):
        main = QVBoxLayout(self)

        # top config
        top_group = QGroupBox("ESP32 연결 설정")
        top_layout = QGridLayout(top_group)

        self.ip_edits = []
        self.conn_labels = []
        self.lamp_widgets = []
        self.status_labels = []

        for i in range(4):
            top_layout.addWidget(QLabel(f"신호등 {i+1} IP"), i, 0)
            edit = QLineEdit()
            edit.setPlaceholderText(f"예: 192.168.0.10{i+1}")
            self.ip_edits.append(edit)
            top_layout.addWidget(edit, i, 1)

            btn_conn = QPushButton(f"{i+1} 연결")
            btn_disc = QPushButton(f"{i+1} 해제")
            btn_status = QPushButton(f"{i+1} 상태조회")

            btn_conn.clicked.connect(lambda _, idx=i: self.connect_node(idx))
            btn_disc.clicked.connect(lambda _, idx=i: self.disconnect_node(idx))
            btn_status.clicked.connect(lambda _, idx=i: self.request_status(idx))

            top_layout.addWidget(btn_conn, i, 2)
            top_layout.addWidget(btn_disc, i, 3)
            top_layout.addWidget(btn_status, i, 4)

            conn_label = QLabel("미연결")
            conn_label.setStyleSheet("font-weight:bold;color:red;")
            self.conn_labels.append(conn_label)
            top_layout.addWidget(conn_label, i, 5)

        main.addWidget(top_group)

        mode_bar = QGroupBox("통합 모드")
        mode_layout = QHBoxLayout(mode_bar)
        self.system_mode_label = QLabel("시스템 모드: MANUAL")
        self.system_mode_label.setStyleSheet("font-size:16px;font-weight:bold;")
        self.active_green_label = QLabel("현재 초록 신호등: -")
        self.active_green_label.setStyleSheet("font-size:16px;font-weight:bold;")
        mode_layout.addWidget(self.system_mode_label)
        mode_layout.addWidget(self.active_green_label)
        mode_layout.addStretch()
        main.addWidget(mode_bar)

        self.tabs = QTabWidget()
        self.auto_tab = QWidget()
        self.manual_tab = QWidget()
        self.tabs.addTab(self.auto_tab, "자동 모드")
        self.tabs.addTab(self.manual_tab, "수동/반자동 모드")
        main.addWidget(self.tabs)

        self.build_auto_tab()
        self.build_manual_tab()

        lamp_group = QGroupBox("신호등 상태")
        lamp_layout = QGridLayout(lamp_group)
        for i in range(4):
            vw = QVBoxLayout()
            lamp = LampWidget(f"Signal {i+1}")
            status = QLabel("STATE: R")
            status.setAlignment(Qt.AlignmentFlag.AlignCenter)
            status.setStyleSheet("font-weight:bold;")
            self.lamp_widgets.append(lamp)
            self.status_labels.append(status)
            vw.addWidget(lamp)
            vw.addWidget(status)
            lamp_layout.addLayout(vw, 0, i)
        main.addWidget(lamp_group)

        log_group = QGroupBox("로그")
        log_layout = QVBoxLayout(log_group)
        self.log_label = QLabel("로그 없음")
        self.log_label.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.log_label.setWordWrap(True)
        self.log_label.setMinimumHeight(180)
        log_layout.addWidget(self.log_label)
        main.addWidget(log_group)

        self.update_global_labels()

    def build_auto_tab(self):
        layout = QVBoxLayout(self.auto_tab)

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

        form.addRow("초록 유지 시간", self.green_spin)
        form.addRow("노랑 유지 시간", self.yellow_spin)

        btn_row = QHBoxLayout()
        self.btn_auto_apply = QPushButton("자동 시간 적용")
        self.btn_auto_start = QPushButton("자동 시작")
        self.btn_auto_stop = QPushButton("자동 중지")
        self.btn_force_init = QPushButton("전체 초기화(R)")
        btn_row.addWidget(self.btn_auto_apply)
        btn_row.addWidget(self.btn_auto_start)
        btn_row.addWidget(self.btn_auto_stop)
        btn_row.addWidget(self.btn_force_init)
        form.addRow(btn_row)

        info = QLabel(
            "자동 순서: 1번 → 2번 → 3번 → 4번 → 1번 반복\n"
            "한 번에 오직 1개 신호등만 초록불이며, 전환 시 현재 초록불은 노랑불 후 빨간불로 바뀐 뒤 다음 신호등이 초록불이 됩니다."
        )
        info.setWordWrap(True)

        layout.addWidget(box)
        layout.addWidget(info)
        layout.addStretch()

        self.btn_auto_apply.clicked.connect(self.apply_auto_config)
        self.btn_auto_start.clicked.connect(self.start_auto_mode)
        self.btn_auto_stop.clicked.connect(self.stop_auto_mode)
        self.btn_force_init.clicked.connect(self.force_all_red)

    def build_manual_tab(self):
        layout = QVBoxLayout(self.manual_tab)

        manual_group = QGroupBox("개별 수동 제어")
        grid = QGridLayout(manual_group)

        self.manual_target_combo = QComboBox()
        for i in range(1, 5):
            self.manual_target_combo.addItem(f"신호등 {i}", i)

        self.blink_interval_spin = QSpinBox()
        self.blink_interval_spin.setRange(100, 5000)
        self.blink_interval_spin.setValue(500)
        self.blink_interval_spin.setSuffix(" ms")

        self.semi_target_combo = QComboBox()
        for i in range(1, 5):
            self.semi_target_combo.addItem(f"신호등 {i}", i)

        self.semi_yellow_spin = QSpinBox()
        self.semi_yellow_spin.setRange(100, 10000)
        self.semi_yellow_spin.setValue(2000)
        self.semi_yellow_spin.setSuffix(" ms")

        grid.addWidget(QLabel("개별 제어 대상"), 0, 0)
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

        layout.addWidget(manual_group)

        semi_group = QGroupBox("반자동 전환")
        semi_layout = QGridLayout(semi_group)
        semi_layout.addWidget(QLabel("다음 초록 대상"), 0, 0)
        semi_layout.addWidget(self.semi_target_combo, 0, 1)
        semi_layout.addWidget(QLabel("노랑 유지 시간"), 0, 2)
        semi_layout.addWidget(self.semi_yellow_spin, 0, 3)

        self.btn_semi_change = QPushButton("현재 초록 → 대상 초록 전환")
        self.btn_set_target_green_now = QPushButton("즉시 대상 G(강제)")
        semi_layout.addWidget(self.btn_semi_change, 1, 0, 1, 2)
        semi_layout.addWidget(self.btn_set_target_green_now, 1, 2, 1, 2)

        self.btn_semi_change.clicked.connect(self.start_semi_auto_change)
        self.btn_set_target_green_now.clicked.connect(self.force_target_green_now)

        note = QLabel(
            "반자동 규칙:\n"
            "- 현재 초록불 신호등이 있으면 먼저 노랑불로 전환\n"
            "- 설정 시간 이후 빨간불로 전환\n"
            "- 그 다음 목표 신호등이 초록불 점등\n"
            "- 현재 초록불이 없으면 바로 목표 신호등을 초록으로 전환"
        )
        note.setWordWrap(True)

        layout.addWidget(semi_group)
        layout.addWidget(note)
        layout.addStretch()

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
        old = self.log_label.text()
        if old == "로그 없음":
            lines = [msg]
        else:
            lines = old.splitlines() + [msg]
        lines = lines[-24:]
        self.log_label.setText("\n".join(lines))

    def node(self, idx_zero):
        return self.nodes[idx_zero]

    def connect_node(self, idx_zero):
        ip = self.ip_edits[idx_zero].text().strip()
        if not ip:
            QMessageBox.warning(self, "경고", f"신호등 {idx_zero+1} IP를 입력하세요.")
            return
        try:
            self.node(idx_zero).connect(ip)
            self.conn_labels[idx_zero].setText(f"연결됨 ({ip})")
            self.conn_labels[idx_zero].setStyleSheet("font-weight:bold;color:green;")
            self.append_log(f"[INFO] 신호등 {idx_zero+1} 연결 성공: {ip}")
            self.request_status(idx_zero)
        except Exception as e:
            self.conn_labels[idx_zero].setText("연결 실패")
            self.conn_labels[idx_zero].setStyleSheet("font-weight:bold;color:red;")
            QMessageBox.critical(self, "연결 오류", f"신호등 {idx_zero+1}: {e}")

    def disconnect_node(self, idx_zero):
        self.node(idx_zero).disconnect()
        self.conn_labels[idx_zero].setText("미연결")
        self.conn_labels[idx_zero].setStyleSheet("font-weight:bold;color:red;")
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
                self.conn_labels[node_id - 1].setStyleSheet("font-weight:bold;color:green;")
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
            self.lamp_widgets[node_id - 1].set_state(lamp)
            self.status_labels[node_id - 1].setText(f"STATE: {lamp} / BLINK: {'ON' if blink else 'OFF'}")
            self.update_global_labels()
            return

        # local OK messages, optimistic state updates
        if line.startswith("OK,SET,"):
            lamp = line.split(",")[-1].strip()
            st.lamp = lamp
            st.blink = False
            self.lamp_widgets[node_id - 1].set_state(lamp)
            self.status_labels[node_id - 1].setText(f"STATE: {lamp} / BLINK: OFF")
            self.update_global_labels()
            return

    def update_global_labels(self):
        self.system_mode_label.setText(f"시스템 모드: {self.system_mode}")
        greens = [n.state.node_id for n in self.nodes if n.state.lamp == "G"]
        if len(greens) == 1:
            self.active_green_label.setText(f"현재 초록 신호등: {greens[0]}")
        elif len(greens) > 1:
            self.active_green_label.setText(f"현재 초록 신호등: 복수({greens})")
        else:
            self.active_green_label.setText("현재 초록 신호등: 없음")

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
        QMessageBox.information(self, "적용", "자동 시간 설정이 적용되었습니다.\n(4개 연동 모드에서는 Red 시간은 자동 계산됩니다.)")

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

        # coordinated auto
        if self.auto_running and self.system_mode == "AUTO":
            elapsed_ms = int((now - self.phase_started) * 1000)

            if self.auto_phase == "GREEN":
                if elapsed_ms >= self.auto_green_ms:
                    self.safe_send(self.current_green, "SET Y")
                    self.auto_phase = "YELLOW"
                    self.phase_started = now

            elif self.auto_phase == "YELLOW":
                if elapsed_ms >= self.auto_yellow_ms:
                    self.safe_send(self.current_green, "SET R")
                    self.current_green = 1 if self.current_green == 4 else self.current_green + 1
                    self.safe_send(self.current_green, "SET G")
                    self.auto_phase = "GREEN"
                    self.phase_started = now
                    self.update_global_labels()

        # semi-auto transfer
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
                self.append_log(
                    f"[INFO] 반자동 전환 완료: {self.transition_from} -> {self.transition_to}"
                )

    def manual_set_selected(self, lamp: str):
        target = self.manual_target_combo.currentData()
        self.auto_running = False
        self.transition_active = False
        self.system_mode = "MANUAL"

        # If setting one signal to G manually, force others to R to keep a valid coordinated system.
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
        if greens:
            return greens[0]
        return None

    def start_semi_auto_change(self):
        target = self.semi_target_combo.currentData()
        yellow_ms = self.semi_yellow_spin.value()

        self.auto_running = False
        self.system_mode = "MANUAL"

        current_green = self.detect_current_green()
        if current_green == target:
            QMessageBox.information(self, "안내", f"신호등 {target}가 이미 초록불입니다.")
            return

        # keep the system valid: all non-participants become red
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
