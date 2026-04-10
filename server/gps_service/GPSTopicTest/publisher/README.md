# 발행(Publisher) — 서버에서 핑키 위치 보내기

이 문서는 **서버(또는 테스트용 PC)** 에서 ROS 2로 “핑키마다 어디 있는지” 같은 정보를 **토픽으로 계속보내는** 방법을 설명합니다.

## 한 줄로 이해하기

- **발행(Publish)** = 라디오 방송처럼, 정해진 **토픽 이름**으로 메시지를 계속 올립니다.
- 핑키(로봇) 쪽은 **같은 토픽**을 **구독**해서 받습니다.
- 서버와 핑키가 **같은 `ROS_DOMAIN_ID`** 를 써야 같은 “방”에서 서로 통신합니다. (숫자만 맞추면 됩니다.)

## 보내는 데이터 내용

- 메시지 타입: `gps_field_msgs/msg/PinkyGps`
- 의미: 바닥(필드)을 **가로 1880 mm, 세로 1410 mm** 직사각형으로 두고, 그 안에서의 위치를 **밀리미터(mm)** 로 표현한 것입니다. (실제 GPS가 아니라 **실내 좌표**에 가깝습니다.)
- 기본 토픽 이름: **`/pinky_{ROS_DOMAIN_ID}/gps_pos`** (예: `export ROS_DOMAIN_ID=23` → `/pinky_23/gps_pos`)

| 항목 | 설명 |
|------|------|
| `pinky_id` | 몇 번 핑키인지 (0, 1, 2 …) |
| `x_mm`, `y_mm` | 필드 안에서의 위치 (mm) |
| `yaw_deg` | 방향 (도) |
| `header` | 시간·좌표계 이름(`frame_id`) 등 |

기본 설정에서는 **`pinky_ids`가 비어 있으면** `ROS_DOMAIN_ID`에 맞는 **한 개의** `pinky_id`만 시뮬합니다. 여러 대를 한 토픽에 올리려면 `-p pinky_ids:="[0,1,2]"` 처럼 지정하면 됩니다.

## 준비물

1. PC에 **ROS 2** 설치 (예: Jazzy, Humble).
2. 이 워크스페이스 **빌드**가 끝난 상태:

`<distro>` 는 설치한 ROS 2 배포판 폴더 이름입니다. 예: `jazzy`, `humble`, `iron`.

```bash
source /opt/ros/<distro>/setup.bash
cd ~/GPSTopicTest
colcon build --symlink-install
source install/setup.bash
```

3. **발행하는 터미널**과 **나중에 구독하는 터미널** 모두에서 **같은** 도메인 ID (예시는 핑키 **23**):

```bash
export ROS_DOMAIN_ID=23
```

> 집/사무실 같은 **같은 PC**에서만 테스트할 때도, 터미널마다 위 줄을 빼먹지 마세요.

> 회사 네트워크처럼 **멀티캐스트가 막힌** 환경에서는 DDS 추가 설정이 필요할 수 있습니다. 먼저 **같은 PC** 또는 **같은 공유기 LAN**에서 시험하는 것을 권장합니다.

## 발행 노드 파라미터 (`field_gps_publisher`)

| 파라미터 | 기본값 | 설명 |
|----------|--------|-----------|
| `topic_name` | `""` | 비어 있으면 `topic_pattern` + **`ROS_DOMAIN_ID`** |
| `topic_pattern` | `/pinky_{domain_id}/gps_pos` | `topic_name`이 비었을 때 |
| `field_width_mm` | `1880.0` | 필드 가로(mm) |
| `field_height_mm` | `1410.0` | 필드 세로(mm) |
| `publish_rate_hz` | `5.0` | 초당 몇 번 “틱”이 도는지(Hz). 틱마다 **pinky_id마다 한 번씩** publish |
| `pinky_ids` | `[]` | 비어 있으면 **`[ROS_DOMAIN_ID & 0xFF]`** 한 대만 시뮬 |
| `frame_id` | `field_1880x1410` | 좌표계 이름 (`header`에 들어감) |

지금 코드는 **데모용으로 필드 안을 도는 가짜 궤적**을 만듭니다. 실제 서비스에서는 `publisher_node.py` 안에서 `x_mm`, `y_mm`를 **진짜 위치**로 바꾸면 됩니다.

## 실행 방법

### 1) 샘플 스크립트 (가장 간단)

```bash
cd ~/GPSTopicTest
colcon build --symlink-install
chmod +x publisher/run_publisher_sample.sh
export ROS_DISTRO=<distro>      # 예: jazzy, humble (위 경로의 폴더 이름과 같게)
export ROS_DOMAIN_ID=23
./publisher/run_publisher_sample.sh
```

뒤에 `--ros-args -p ...` 같은 인자를 붙이면 그대로 `ros2 run`에 전달됩니다.

### 2) `ros2 run` 직접

```bash
source ~/GPSTopicTest/install/setup.bash
export ROS_DOMAIN_ID=23
ros2 run gps_field_publisher field_gps_publisher
```

### 3) 파라미터 바꿔서 실행

```bash
ros2 run gps_field_publisher field_gps_publisher --ros-args \
  -p pinky_ids:="[0,1,2,5]" \
  -p publish_rate_hz:=10.0
```

쉘에서 `pinky_ids` 배열이 잘 안 먹으면, **launch 파일**에 정수 배열로 적는 편이 편합니다.

### 4) Launch 파일

```bash
ros2 launch gps_field_publisher field_gps_publisher.launch.py
ros2 launch gps_field_publisher field_gps_publisher.launch.py topic_name:=/my_field/gps
```

## 서버에서 “잘 나가는지” 확인

**다른 터미널**을 열고 (역시 `ROS_DOMAIN_ID` 동일):

```bash
source ~/GPSTopicTest/install/setup.bash
export ROS_DOMAIN_ID=23
ros2 topic list | grep pinky
ros2 topic echo /pinky_23/gps_pos --no-arr
```

`pinky_id`, `x_mm`, `y_mm`가 주기적으로 바뀌면 **발행은 정상**입니다.

## 구독 측(핑키)과 맞출 체크리스트

1. **`ROS_DOMAIN_ID` 동일**
2. **토픽 이름 동일** (`topic_name` / launch 인자)
3. 구독 쪽에서 고른 **`pinky_id`** 가 발행 쪽 **`pinky_ids` 목록 안에 있을 것**  
   (없으면 구독 창은 계속 “대기”만 합니다.)

구독·LCD 쪽은 **`subscriber/README.md`** 를 보세요.

## 소스 위치

- 패키지: `~/GPSTopicTest/src/gps_field_publisher/`
- 노드: `gps_field_publisher/publisher_node.py`

## 자주 생기는 문제

| 증상 | 확인할 것 |
|------|-----------|
| `AMENT_TRACE_SETUP_FILES: unbound variable` (샘플 스크립트) | `publisher/run_publisher_sample.sh` 최신본 사용 (`set -u` 제거됨). |
| `ros2 topic list`에 토픽이 없다 | `source install/setup.bash` 했는지, `ROS_DOMAIN_ID` 맞는지, 발행 노드가 켜져 있는지 |
| echo는 되는데 핑키 창이 안 갱신된다 | 구독 쪽 도메인·토픽·`pinky_id` 불일치 |
| 메시지 타입 빌드 오류 | `gps_field_msgs` 포함해서 `colcon build` 전체 다시 |
