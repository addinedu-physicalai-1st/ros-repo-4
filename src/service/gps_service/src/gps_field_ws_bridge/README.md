# WebSocket → `PinkyGps` 발행 브리지

**클라이언트(WebSocket)** ↔ **`ws_gps_publisher`(서버)** → **`/pinky_{pinky_id}/gps_pos`** (`gps_field_msgs/PinkyGps`)

## 개념 (네임스페이스 스타일)

- 토픽 이름의 숫자는 **`pinky_id` (또는 `ros_main_id`)** 를 기준으로 결정됩니다.
- 메시지 타입에 **`pinky_id`** 필드는 남아 있습니다. JSON에 생략하면 서버가 **`ros_main_id`가 있으면 그 값(하위 바이트)**, 없으면 **`ROS_DOMAIN_ID & 0xFF`** 로 채웁니다. 구독 노드는 **`pinky_id:=N`(필터)** 를 쓰면 `msg.pinky_id == N` 인 것만 표시합니다.

### `ROS_DOMAIN_ID`를 안 썼는데 LCD에 반영되는 이유

**설정을 안 하면 “도메인 없음”이 아니라, 보통 도메인 `0` 입니다.**  
`ws_gps_publisher`·LCD용 구독 노드를 **같은 머신에서** `export` 없이 띄우면 둘 다 기본값 **0** → 토픽은 **`/pinky_0/gps_pos`** 로 맞고, DDS에서도 같은 도메인이라 **서로 보입니다.**  
도메인으로 격리해 테스트하려면 터미널마다 **의도적으로 다른 값**(예: 서버·LCD는 23, 다른 발행만 22)을 `export ROS_DOMAIN_ID=...` 하세요.

## 의존성·빌드

```bash
cd ~/GPSTopicTest
source /opt/ros/<distro>/setup.bash
colcon build --packages-select gps_field_ws_bridge
source install/setup.bash
```

## 서버 실행 (먼저 띄우기)

```bash
source /opt/ros/<distro>/setup.bash
source ~/GPSTopicTest/install/setup.bash
export ROS_DOMAIN_ID=23
ros2 run gps_field_ws_bridge ws_gps_publisher
```

→ **`/pinky_23/gps_pos`** 로 publish (패턴은 파라미터로 변경 가능).

| 파라미터 | 기본 | 설명 |
|----------|------|------|
| `topic_name` | `""` | 비어 있으면 `topic_pattern`으로 토픽 생성. **비어 있지 않으면** 원칙적으로 고정 토픽만 사용(JSON `ros_main_id` 무시). **다만** `pinky_ids_env_file` 로 읽은 id 목록이 **비어 있지 않으면** `.env` 다중 발행이 우선하고 이 경우 `topic_name` 은 **무시**됨(런치에 고정 토픽이 있어도 핑키 리스트로 여러 토픽 발행 가능) |
| `topic_pattern` | `/pinky_{pinky_id}/gps_pos` | `topic_name` 고정을 쓰지 않을 때(및 `.env` 목록의 각 id에 대해) 토픽 문자열 생성 |
| `topic_domain_from_env_only` | `false` | `true` 이면 토픽 숫자는 **항상 서버의 `ROS_DOMAIN_ID`** 만 사용. JSON `ros_main_id` 는 **토픽이 아니라** `pinky_id` 등 메시지 필드 정렬용(LCD는 `export ROS_DOMAIN_ID`만 맞추면 됨). `false`(기본)이면 JSON `ros_main_id` 가 있을 때 `/pinky_{ros_main_id}/gps_pos` 로 분기 |
| `pinky_ids_env_file` | `""` | **비어 있지 않으면** 해당 경로의 `.env` 를 읽음. 목록이 파싱되면 **WebSocket 메시지 한 건당** `topic_pattern` 의 id 마다 **동일 `PinkyGps`** 를 publish (`topic_name` 이 있어도 **목록이 우선**). JSON `ros_main_id` / `topic_domain_from_env_only` 보다 **우선**. 상대 경로는 **노드 실행 시 작업 디렉터리** 기준. **`.env` 가 없으면** `share/.../config/.env.example` 로 자동 폴백(파라미터가 `.env` 일 때) |
| `pinky_ids_env_key` | `PINKY_GPS_PUBLISH_IDS` | `.env` 안에서 id 목록 문자열을 담은 키 이름 (예: `14,24,34` 또는 `14; 24; 34`) |
| `pinky_ids_env_filter_by_pinky_id` | `false` | `true` 이면 `.env` 목록은 **허용 집합**만 두고, JSON에 `ros_main_id`(관례상 pinky_id와 동일)가 있으면 **그 id에 해당하는 토픽 한 곳**만 발행(목록에 없으면 생략). `ros_main_id`가 없으면 기존처럼 목록 **전체**에 브로드캐스트 |

레거시 호환(Deprecated): `domain_ids_env_key`, `domain_ids_env_filter_by_ros_main_id` 도 그대로 동작하지만, 신규 이름 사용을 권장합니다.
| `ws_port` | `8765` | |

**`.env` 다중 발행 예** (`ws_gps_publisher` 를 `GPSTopicTest` 에서 실행한다고 가정):

```env
# .env
PINKY_GPS_PUBLISH_IDS=1,2,3
```

```bash
ros2 run gps_field_ws_bridge ws_gps_publisher --ros-args -p pinky_ids_env_file:=.env
```

**패키지에 포함된 예시 파일:** 빌드 후 `share/gps_field_ws_bridge/config/.env.example`  
소스 트리: `src/gps_field_ws_bridge/config/.env.example`

```bash
# 작업 디렉터리에 .env 로 복사 후 수정
cp install/gps_field_ws_bridge/share/gps_field_ws_bridge/config/.env.example .env
# 또는 (source 워크스페이스에서)
cp src/gps_field_ws_bridge/config/.env.example .env
```

**오해 방지:** ROS 2에서 토픽 이름은 미리 “등록”될 필요가 없습니다. 다만 **퍼블리시하는 문자열**과 **구독하는 문자열**이 같아야 하고, 노드들의 **DDS `ROS_DOMAIN_ID`** 도 같아야 합니다. “없는 토픽 id”처럼 보이면 구독이 `/pinky_0/...` 인데 퍼블시가 `/pinky_23/...` 인 경우가 많습니다.

## 클라이언트 JSON

필수: **`x_mm`**, **`y_mm`**  
선택: **`yaw_deg`**, **`frame_id`**, **`pinky_id`** (없으면 `ros_main_id`와 동일하게 맞춤, 그것도 없으면 서버 `ROS_DOMAIN_ID` 하위 바이트), **`ros_main_id`** (정수 → publish 토픽을 `/pinky_{ros_main_id}/gps_pos` 로; 없으면 서버 환경 `ROS_DOMAIN_ID` 사용. **DDS 도메인은 서버·구독 프로세스의 `ROS_DOMAIN_ID`로 격리**되고, JSON 값은 **토픽 이름의 숫자만** 바꿉니다 — **구독 쪽도 같은 토픽 문자열을 써야** 합니다.)

```json
{"x_mm": 500.0, "y_mm": 400.0, "yaw_deg": 0.0, "frame_id": "field_1880x1410", "ros_main_id": 22}
```

데모 CLI: **`--ros-main-id N`** 이 있으면 JSON에 넣음. 없으면 클라이언트 셸의 **`export ROS_DOMAIN_ID`** 값을 넣고, 그것도 없으면 `ros_main_id` 키를 생략합니다.

## 구독 + LCD (`field_gps_subscriber`)

같은 기기에서 **`ROS_DOMAIN_ID` 동일** + **`topic_name` 비우기**(기본)면 자동으로 **`/pinky_23/gps_pos`** 를 구독합니다.

```bash
export ROS_DOMAIN_ID=23
source ~/GPSTopicTest/install/setup.bash
ros2 run gps_field_subscriber field_gps_subscriber --ros-args \
  -p show_tk:=false \
  -p show_oled:=false \
  -p show_pinky_spi:=true
```

`pinky_id` 필터를 쓰려면 `-p pinky_id:=5` 처럼 **0 이상**으로 주면 됩니다.

## 데모 클라이언트

### 토픽으로 올라갈 샘플 **1건**만 보내기 (`--loop` 없음)

```bash
source ~/GPSTopicTest/install/setup.bash
ros2 run gps_field_ws_bridge ws_gps_client_demo -- \
  --uri ws://127.0.0.1:8765 --x 940 --y 705 --yaw 90 --frame-id field_1880x1410 \
  --ros-main-id 22
# 또는 --ros-main-id 생략 후 export ROS_DOMAIN_ID=23 이 있으면 그 숫자가 JSON에 들어감
# 선택: --pinky-id 0
```

→ WebSocket으로 JSON 한 번 전송 후 종료. 서버가 `PinkyGps` 한 번 publish.

### 연속 전송 (사인 경로)

```bash
cd ~/GPSTopicTest/src/gps_field_ws_bridge
PYTHONPATH="$PWD" python3 -m gps_field_ws_bridge.ws_gps_client_demo \
  --uri ws://127.0.0.1:8765 --loop --rate 5
```

`ros2 run` 사용 시에만 인자 앞에 `--` 가 필요합니다.

**`Package not found`**: `source install/setup.bash` 후 실행.  
**`Connection refused`**: 서버(`ws_gps_publisher`)를 **먼저** 실행.  
**`unrecognized arguments: --ros-main-id`**: 소스 수정 후 `colcon build --packages-select gps_field_ws_bridge` 하고 `source install/setup.bash` 다시 하세요.

### LCD / 구독이 안 바뀔 때 (`--ros-main-id`만 켠 경우)

JSON의 **`ros_main_id`는 브리지가 퍼블리시하는 토픽 이름만 바꿉니다.**  
`field_gps_subscriber`는 **`topic_name`이 비어 있으면** 구독 토픽을 **항상 자기 셸의 `ROS_DOMAIN_ID`** 로만 만듭니다 (미설정이면 **0** → `/pinky_0/gps_pos`).

그래서 **`--ros-main-id 23`으로내면** 메시지는 **`/pinky_23/gps_pos`**로만 올라가고, LCD가 **`export ROS_DOMAIN_ID` 없이** 떠 있으면 **`/pinky_0/gps_pos`**만 듣고 있어 **화면이 그대로**인 것이 정상입니다.

**해결 (하나만 하면 됨):**

- LCD/구독 터미널에서 **`export ROS_DOMAIN_ID=23`** 후 구독 노드 재실행, 또는  
- 구독에 **`-p topic_name:=/pinky_23/gps_pos`** 를 명시.

같은 머신에서 DDS로 서로 보이게 하려면, 퍼블리셔·구독·기타 노드의 **`ROS_DOMAIN_ID`는 보통 동일**하게 두는 것이 안전합니다 (격리 테스트가 아니라면).

확인: 퍼블리셔와 **동일한** `source`·`ROS_DOMAIN_ID`로  
`ros2 topic echo /pinky_23/gps_pos` — 여기에 값이 찍히면 브리지는 정상이고 구독 토픽만 맞추면 됩니다.

## Pinky에서 통합 테스트 (도메인 23)

1. 세 터미널 모두: `export ROS_DOMAIN_ID=23` + `source install/setup.bash`
2. **A**: `ros2 run gps_field_ws_bridge ws_gps_publisher`
3. **B**: `ros2 run gps_field_subscriber field_gps_subscriber --ros-args -p show_pinky_spi:=true -p show_tk:=false -p show_oled:=false`
4. **C**: `PYTHONPATH=... python3 -m gps_field_ws_bridge.ws_gps_client_demo --uri ws://127.0.0.1:8765 --loop --rate 5`

확인: `ros2 topic echo /pinky_23/gps_pos --no-arr`

## 보안

로컬/내부망용 예제입니다. 인터넷에 노출 시 인증·TLS(`wss`)·방화벽을 별도로 적용하세요.
