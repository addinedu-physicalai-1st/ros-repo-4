# dalimi_GPS_GUI3

`dalimi_GPS_GUI2`와 동일한 맵 코너(mm) 호모그래피·다중 핑키·스냅샷 흐름을 유지하면서, **타이밍만 테스트용으로 분리**한 버전입니다.

## GUI2 대비 차이

| 항목 | GUI2 | GUI3 |
|------|------|------|
| 카메라 타이머 | 고정 20ms | **목표 FPS** (`CAMERA_TARGET_FPS`, 기본 30 → ~33ms) |
| Pos 에디터 | 매 오버레이마다 갱신 | 동일 (**프레임마다** 인식 직후 갱신) |
| WebSocket | 별도 50ms 타이머 + `WS_RATE_HZ` 스로틀 | **N프레임마다** 전송 (`WS_SEND_EVERY_N_FRAMES`), 송신 스레드는 **시간 스로틀 없음** |

실제 카메라가 30fps가 아니어도, 타이머는 설정한 목표 FPS에 맞춰 `read()`·검출을 **요청**합니다. 처리가 느리면 누적 지연이 날 수 있습니다.

## 실행

Ubuntu/Debian에서는 보통 **`python` 대신 `python3`** 만 있습니다.

```bash
cd ~/tmp_cursor_project/dalimi_GPS_GUI3

# 가상환경(권장): 이전에 실패했다면 rm -rf .venv 후 다시 생성
python3 -m venv .venv
source .venv/bin/activate

# 아래 두 줄이 venv 안으로 들어가야 합니다. "Defaulting to user installation" 이면 venv가 깨졌거나 activate가 안 된 것
which python3    # .../dalimi_GPS_GUI3/.venv/bin/python3 여야 함
python3 -m pip install -U pip
python3 -m pip install -r requirements.txt
python3 -m pip install -e .

# 프로젝트 루트에서 실행(.env는 루트에 두면 됨)
python3 -m dalimi_gps_gui3
```

`.env`가 없으면 실행 시 `.env.example`을 복사해 생성합니다.

### 자주 나는 증상

| 증상 | 대응 |
|------|------|
| `Command 'python' not found` | 위처럼 **`python3`** 사용 |
| `Defaulting to user installation...` | `deactivate` → `rm -rf .venv` → `python3 -m venv .venv` → 다시 `activate` 후 **`python3 -m pip`** 로 설치 |
| `No module named dalimi_gps_gui3` | 프로젝트 루트에서 `python3 -m pip install -e .` 실행 |

## 환경 변수

- `CAMERA_TARGET_FPS` (기본 30)
- `WS_SEND_EVERY_N_FRAMES` (기본 5 → 약 6회/초 @ 30fps 처리 시)

ROS 브리지(`ws_gps_publisher`) 사용법은 `GPSTopicTest` 문서와 동일합니다.
