# 구독(Subscriber) — 핑키에서 위치 받아 LCD에 보이기

이 문서는 **핑키(로봇 PC)** 등에서 기본 **`/pinky_{pinky_id}/gps_pos`** 토픽을 **구독**하고, (선택) **`pinky_id`로 메시지를 걸러** **데스크톱 창(Tk)** · **I2C OLED** · **전면 SPI TFT (`pinky_lcd`)** 에 표시하는 방법을 설명합니다.

발행(서버) 쪽 설명은 **`publisher/README.md`** 를 보세요.

## 한 줄로 이해하기

- **구독(Subscribe)** = 서버가 올리는 **같은 토픽**을 듣습니다.
- 기본 **`pinky_id:=-1`** 이면 토픽에 오는 메시지를 모두 표시합니다. **0 이상**으로 두면 **`msg.pinky_id`와 같은 메시지**만 반영합니다.
- 서버와 **같은 `ROS_DOMAIN_ID`** 가 필수입니다. (이 문서 예시: **23** — 핑키에 맞게 통일하세요.)

## 준비물

`<distro>` 는 설치한 ROS 2 배포판 폴더 이름입니다. 예: `jazzy`, `humble`, `iron`.

```bash
source /opt/ros/<distro>/setup.bash
cd ~/GPSTopicTest
colcon build --symlink-install
source install/setup.bash
```

**발행 쪽과 동일한 도메인:**

```bash
export ROS_DOMAIN_ID=23
```

**Tk 창**을 쓰려면 `DISPLAY`가 필요합니다 (모니터, `ssh -X`, VNC 등). **OLED만** 또는 **Pinky SPI LCD만** 쓰면 데스크톱 없이도 가능합니다.

```bash
sudo apt install python3-tk    # Tk 쓸 때만 (Debian/Ubuntu 예시)
```

## 구독 노드 파라미터 (`field_gps_subscriber`)

| 파라미터 | 기본값 | 설명 |
|----------|--------|-----------|
| `topic_name` | `""` | 비어 있으면 `topic_pattern` + **`pinky_id`** → 기본 `/pinky_N/gps_pos` |
| `topic_pattern` | `/pinky_{pinky_id}/gps_pos` | `topic_name`이 비었을 때만 사용 |
| `pinky_id` | `-1` | **`-1`**: 토픽에 오는 메시지 모두 표시. **`0` 이상**: `msg.pinky_id`와 같은 것만 표시 |
| `field_width_mm` | `1880.0` | 창 제목에만 표시 (안내용) |
| `field_height_mm` | `1410.0` | 창 제목에만 표시 (안내용) |
| `show_tk` | `true` | 데스크톱 **Tk** 창 사용 여부 |
| `show_oled` | `false` | **I2C OLED** 사용 여부 (`luma.oled` 필요) |
| `oled_i2c_port` | `1` | I2C 버스 번호 (라즈베리파이는 보통 `1`) |
| `oled_i2c_address` | `60` | 7비트 주소의 **10진수** (`0x3C` → `60`, `0x3D` → `61`) |
| `oled_device` | `ssd1306` | 칩 종류: `ssd1306` 또는 `sh1106` |
| `oled_rotate` | `0` | `luma` 회전 값 (필요 시 0/1/2/3) |
| `oled_width` | `128` | 가로 픽셀 (128x32 모듈이면 128) |
| `oled_height` | `64` | 세로 픽셀 (**128x32 OLED면 `32`**) |
| `oled_contrast` | `255` | `0`이면 대비 설정 생략, 어두우면 255 유지 |
| `show_pinky_spi` | `false` | **전면 SPI TFT** (`from pinky_lcd import LCD`, PIL `img_show`와 동일 경로) |
| `pinky_lcd_img_width` | `320` | 노트북 예제와 같이 만들 이미지 가로 |
| `pinky_lcd_img_height` | `240` | 노트북 예제와 같이 만들 이미지 세로 |

## WebSocket 브리지 (`gps_field_ws_bridge`) 와 함께 쓸 때

브리지와 구독 모두 **`export ROS_DOMAIN_ID`** 를 맞추면 기본 토픽 **`/pinky_N/gps_pos`** 가 같아집니다. **`topic_name`을 비워 두면** 자동으로 그 경로를 씁니다.

- **`pinky_id:=-1`(기본)** 이면 메시지의 `pinky_id` 필터 없이 표시합니다.
- 레거시 토픽(`/pinky_field/gps` 등)을 쓰려면 `-p topic_name:=/그/경로` 로 고정하면 됩니다.

자세한 실행 예: `src/gps_field_ws_bridge/README.md`

## Pinky 전면 SPI LCD (`pinky_lcd`)

노트북에서 아래처럼 잘 되던 패널이면, 구독 노드에서도 **같은 `LCD()` + `img_show`** 로 GPS를 그립니다.

- **필요**: 보드에 이미 설치된 `pinky_lcd` 패키지, `PIL`(Pillow), SPI/GPIO 접근 권한(보통 핑키 이미지에 포함).
- ROS 패키지 `package.xml`에 `pinky_lcd`를 `exec_depend`로 넣기 어려운 경우가 많으므로, **로봇 환경에서 `python3 -c "from pinky_lcd import LCD"`** 가 되는지 먼저 확인하세요.

**SPI LCD만** (Tk·OLED 끄기):

```bash
source ~/GPSTopicTest/install/setup.bash
export ROS_DOMAIN_ID=23
ros2 run gps_field_subscriber field_gps_subscriber --ros-args \
  -p pinky_id:=0 \
  -p show_tk:=false \
  -p show_oled:=false \
  -p show_pinky_spi:=true
```

**Launch** 예:

```bash
ros2 launch gps_field_subscriber field_gps_subscriber.launch.py \
  pinky_id:=0 show_tk:=false show_oled:=false show_pinky_spi:=true
```

## I2C OLED (SSD1306 / SH1106) 같이 쓰기

1. **I2C 켜기** (라즈베리파이 예): `sudo raspi-config` → Interface Options → I2C 활성화.  
2. **배선**: OLED `VCC/GND/SDA/SCL`을 Pi의 `3.3V/GND/SDA1/SCL1`에 연결 (일반 128x64 I2C 모듈).  
3. **주소 확인**:

```bash
sudo apt install -y i2c-tools
i2cdetect -y 1
```

표에 `UU` 또는 `3c` / `3d` 가 보이면 그 주소를 `oled_i2c_address`에 **10진수**로 넣습니다 (`3c` → `60`).

4. **파이썬 패키지** (ROS와 **같은** `python3`):

```bash
cd ~/GPSTopicTest/src/gps_field_subscriber
python3 -m pip install --user -r requirements-oled.txt
```

또는 `pip install luma.oled`.

5. **권한**: I2C 접근을 위해 사용자를 `i2c` 그룹에 넣고 재로그인합니다.

```bash
sudo usermod -aG i2c $USER
```

6. **Tk + OLED 동시** 실행 예:

```bash
source ~/GPSTopicTest/install/setup.bash
export ROS_DOMAIN_ID=23
ros2 run gps_field_subscriber field_gps_subscriber --ros-args \
  -p pinky_id:=0 \
  -p show_tk:=true \
  -p show_oled:=true \
  -p oled_i2c_port:=1 \
  -p oled_i2c_address:=60
```

**OLED만** (모니터 없이):

```bash
ros2 run gps_field_subscriber field_gps_subscriber --ros-args \
  -p pinky_id:=0 \
  -p show_tk:=false \
  -p show_oled:=true
```

이 경우 노드는 **메인 스레드에서 `spin`** 하므로 터미널은 그대로 두면 됩니다. 종료는 `Ctrl+C`.

**Launch** 예:

```bash
ros2 launch gps_field_subscriber field_gps_subscriber.launch.py \
  pinky_id:=0 show_tk:=true show_oled:=true oled_i2c_address:=60
```

## 실행 방법

### 1) 샘플 스크립트

한 토픽에 여러 `pinky_id`가 섞일 때 **1번만** 보고 싶으면 `-p pinky_id:=1` 을 씁니다. 기본 필터 없음 예:

```bash
cd ~/GPSTopicTest
source /opt/ros/<distro>/setup.bash
colcon build --symlink-install
export ROS_DOMAIN_ID=23
PINKY_ID=1 ./subscriber/run_subscriber_sample.sh
```

### 2) `ros2 run`

```bash
source ~/GPSTopicTest/install/setup.bash
export ROS_DOMAIN_ID=23
ros2 run gps_field_subscriber field_gps_subscriber --ros-args -p pinky_id:=1
```

### 3) Launch

```bash
ros2 launch gps_field_subscriber field_gps_subscriber.launch.py pinky_id:=1
```

## 같은 PC에서 끝까지 테스트 (추천 순서)

**터미널 A — 발행만**

```bash
source ~/GPSTopicTest/install/setup.bash
export ROS_DOMAIN_ID=23
ros2 run gps_field_publisher field_gps_publisher
```

**터미널 B — 구독 (0번만 표시)**

```bash
source ~/GPSTopicTest/install/setup.bash
export ROS_DOMAIN_ID=23
ros2 run gps_field_subscriber field_gps_subscriber --ros-args -p pinky_id:=0
```

창에 `x_mm`, `y_mm`, `yaw`, `stamp`가 계속 바뀌면 성공입니다.

## 서버는 멀리 있고, 핑키만 따로 있을 때

1. **양쪽 OS 터미널/서비스**에서 `ROS_DOMAIN_ID`를 **똑같이**.
2. 네트워크에서 ROS 2 DDS **멀티캐스트**가 통하는지 (방화벽, 서브넷).
3. 서버 PC와 핑키 PC **각각**에서 `ros2 topic list` 했을 때 **같은 토픽 이름**이 보이는지 확인.

안 보이면 ROS 2 문서의 DDS / `FASTRTPS` 설정을 추가로 검토해야 합니다.

## 화면 없이 “값만” 보고 싶을 때

GUI 없이 터미널에서 확인:

```bash
ros2 topic echo /pinky_23/gps_pos --no-arr
```

## 소스 위치

- 패키지: `~/GPSTopicTest/src/gps_field_subscriber/`
- 노드: `gps_field_subscriber/subscriber_node.py`
- OLED 드라이버 래퍼: `gps_field_subscriber/oled_display.py` (`luma.oled`)
- OLED 점검 CLI: `field_oled_selftest` (`gps_field_subscriber/oled_selftest.py`)

## OLED(전면 LCD)가 **아예 안 나올 때** (가장 흔한 원인)

1. **`i2cdetect`로 주소 확인** (버스 1이 안 되면 `0`도 시험):

```bash
i2cdetect -y 1
```

- **SSD1306/SH1106 I2C OLED**는 보통 **`3c` 또는 `3d`** 가 표에 보입니다.  
  이 경우 `oled_i2c_address:=60` (0x3C) 또는 `61` (0x3D) 로 맞춥니다.
- 표에 **`3c`/`3d`가 전혀 없고** 다른 값만 보이면, 전면 LCD가 **이 코드와 다른 버스/다른 칩(SPI·DSI·HDMI 등)** 일 수 있습니다. 보드 매뉴얼로 인터페이스를 확인하세요.

2. **ROS 없이 OLED만 검사** (`i2cdetect` + **버스 스캔으로 보이는 주소 전부**에 대해 `ssd1306`/`sh1106` 시도):

```bash
source ~/GPSTopicTest/install/setup.bash
ros2 run gps_field_subscriber field_oled_selftest
```

- `3c`/`3d`가 **없고** `6b`, `08` 같은 값만 있으면: 그 주소는 **대개 OLED(SSD1306)가 아닙니다.** (`6b`는 IMU 등 다른 칩인 경우가 많음.)  
  이 경우 **전면 LCD가 SPI·DSI·다른 버스**일 수 있어, **이 I2C + luma 방식으로는 절대 안 켜질 수 있습니다.**
- `Draw OK`가 나온 **10진 주소**만 `oled_i2c_address`에 넣으면 됩니다.
- **버스 0**도 의심되면:  
  `ros2 run gps_field_subscriber field_oled_selftest -- --port 0`  
  또는 `--port0-also` 로 0번 버스까지 스캔합니다.

추가 주소만 지정:

```bash
ros2 run gps_field_subscriber field_oled_selftest -- --addresses 107 --height 32
```

3. **모듈이 128x32** 이면 `-p oled_height:=32` 를 꼭 넣으세요.

4. 로그에 `OLED init failed` 가 있으면 `luma.oled` 미설치 또는 **잘못된 I2C 주소**입니다. `oled=True` 인데도 화면이 검정이면 **주소/높이(32)/sh1106** 을 다시 맞춥니다.

## 자주 생기는 문제

| 증상 | 조치 |
|------|------|
| `AMENT_TRACE_SETUP_FILES: unbound variable` (샘플 스크립트) | 스크립트에서 이미 수정됨. 예전 버전이면 `subscriber/run_subscriber_sample.sh` 를 최신으로 두거나, `set -u` 없이 `source` 하세요. |
| Tkinter / DISPLAY 오류 | `echo $DISPLAY`, `ssh -X`, `python3-tk` 설치 |
| 창은 뜨는데 계속 “Waiting” | `pinky_id`가 발행 목록에 없음, `ROS_DOMAIN_ID` 다름, 토픽 이름 다름 |
| `gps_field_msgs` import 오류 | `source install/setup.bash`, 빌드 다시 |
| OLED가 안 켜지거나 `luma` 오류 | `pip install luma.oled`, I2C 활성화, `i2cdetect`, `i2c` 그룹, **`field_oled_selftest`로 주소 확인** |
| Permission denied I2C | `sudo`로 실행하지 말고 `i2c` 그룹 + 재로그인 |

발행 설정은 **`publisher/README.md`** 를 참고하세요.
