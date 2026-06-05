# 팀원별 핑키 GPS 구독 가이드

같은 필드에서 여러 대의 핑키가 있을 때, **각 사람은 자신에게 배정된 `pinky_id`에 해당하는 토픽만** 구독하면 됩니다. 이 문서는 `GPSTopicTest` 워크스페이스(`gps_field_ws_bridge`, `gps_field_subscriber`) 기준입니다.

---

## 1. 꼭 구분할 두 가지

| 항목 | 의미 | 예시 |
|------|------|------|
| **`ROS_DOMAIN_ID`** | ROS 2 DDS가 메시지를 주고받는 **네트워크 격리 번호**. 필드에 있는 PC·라즈베리파이·브리지 서버가 **같이 통신하려면 모두 동일**해야 합니다. | `23` |
| **`pinky_id`** | 토픽 이름과 메시지 안의 **로봇 식별 번호**. 사람마다 **다른 값**을 씁니다. | `14`, `24`, `34`, … |

- 토픽 이름(기본): **`/pinky_{pinky_id}/gps_pos`**
- 예: 24번 핑키만 보려면 구독 토픽은 **`/pinky_24/gps_pos`** 입니다.
- **`ROS_DOMAIN_ID=23`이고 `pinky_id=24`인 것은 정상**입니다. (도메인 23 안에서 24번 토픽을 쓰는 구조)

---

## 2. 사전 준비 (한 번만)

1. 저장소 클론 또는 공유 폴더에서 워크스페이스로 이동합니다.

   ```bash
   cd ~/tmp_cursor_project/GPSTopicTest   # 실제 경로에 맞게 수정
   ```

2. ROS 2와 워크스페이스를 소싱한 뒤 빌드합니다. (배포판이 **Jazzy**인 경우)

   ```bash
   source /opt/ros/jazzy/setup.bash
   colcon build
   source install/setup.bash
   ```

3. 필드 쪽에서 **WebSocket → ROS 브리지**(`ws_gps_publisher`)가 이미 떠 있고, `dalimi_GPS_GUI2` 등에서 해당 `pinky_id`로 JSON이 나가는지 확인합니다.  
   브리지는 보통 `.env`의 `PINKY_GPS_PUBLISH_IDS`에 팀 전체 id가 들어 있습니다.

---

## 3. 내 핑키만 보기 — LCD(Tk) 구독 노드

터미널에서 **필드와 같은 `ROS_DOMAIN_ID`**를 쓰고, **내 `pinky_id`**만 넘깁니다.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/tmp_cursor_project/GPSTopicTest
source install/setup.bash

export ROS_DOMAIN_ID=23          # 필드 전체와 동일한 값으로 맞출 것
ros2 run gps_field_subscriber field_gps_subscriber --ros-args -p pinky_id:=24
```

- `24`를 **본인에게 배정된 번호**로 바꿉니다.
- 창 제목·하단에 **`PINKY_ID=24`**, 토픽 **`/pinky_24/gps_pos`** 가 표시됩니다.

### OLED / SPI LCD만 쓰는 경우

```bash
ros2 run gps_field_subscriber field_gps_subscriber --ros-args \
  -p pinky_id:=24 \
  -p show_tk:=false \
  -p show_oled:=true
```

(하드웨어에 맞게 `oled_*` 파라미터를 조정합니다.)

---

## 4. Launch로 실행 (선택)

```bash
export ROS_DOMAIN_ID=23
ros2 launch gps_field_subscriber field_gps_subscriber.launch.py pinky_id:=24
```

`topic_name`을 비우면 기본적으로 **`/pinky_{pinky_id}/gps_pos`** 를 구독합니다.

---

## 5. 토픽만 확인하고 싶을 때 (echo / list)

```bash
export ROS_DOMAIN_ID=23
source /opt/ros/jazzy/setup.bash
source ~/tmp_cursor_project/GPSTopicTest/install/setup.bash

ros2 topic list | grep gps_pos
ros2 topic echo /pinky_24/gps_pos --once
```

메시지 타입: `gps_field_msgs/msg/PinkyGps`

---

## 6. 자주 나는 실수

1. **`ROS_DOMAIN_ID` 불일치**  
   브리지 PC는 `23`, 내 PC는 설정 안 함(기본 `0`) → 서로 DDS에서 안 보입니다. **필드와 동일하게 `export`** 하세요.

2. **`pinky_id`만 맞추고 토픽은 다른 번호**  
   구독 노드는 **`pinky_id` 파라미터로 토픽 이름을 만듭니다.** 예전처럼 `ROS_DOMAIN_ID` 숫자로만 토픽이 정해지지 않습니다.

3. **브리지에 내 id가 없음**  
   `ws_gps_publisher`의 `.env`에 `PINKY_GPS_PUBLISH_IDS`에 본인 id가 포함되어 있어야 해당 토픽으로 publish 됩니다.

4. **소스만 수정하고 `install` 미반영**  
   코드 수정 후에는 `colcon build` 후 **`source install/setup.bash`** 를 다시 하세요.

---

## 7. 부팅 후 자동 실행 (systemd 예시, Jazzy)

`/etc/systemd/system/gps_field_subscriber_pinky24.service` 예:

```ini
[Unit]
Description=Pinky GPS subscriber (pinky_id=24)
After=network.target

[Service]
Type=simple
User=YOUR_USER
WorkingDirectory=/home/YOUR_USER/tmp_cursor_project/GPSTopicTest
Environment=ROS_DOMAIN_ID=23
ExecStart=/bin/bash -lc "source /opt/ros/jazzy/setup.bash && source /home/YOUR_USER/tmp_cursor_project/GPSTopicTest/install/setup.bash && ros2 run gps_field_subscriber field_gps_subscriber --ros-args -p pinky_id:=24"
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
```

`YOUR_USER`, 경로, `ROS_DOMAIN_ID`, `pinky_id`를 본인 환경에 맞게 수정합니다.

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now gps_field_subscriber_pinky24.service
```

GUI(Tk)를 서비스로 띄우려면 디스플레이·세션 변수(`DISPLAY`, Wayland 등)가 필요할 수 있어, 데스크톱 자동 로그인 후 **사용자 세션용 autostart**가 더 나을 때도 있습니다.

---

## 8. 요약 체크리스트

- [ ] 필드와 동일한 **`ROS_DOMAIN_ID`**
- [ ] 본인 **`pinky_id`** 로 `field_gps_subscriber` 실행
- [ ] 구독 토픽이 **`/pinky_<내_id>/gps_pos`** 인지 확인
- [ ] `ws_gps_publisher`가 해당 id로 publish 하는지 확인

더 자세한 브리지·JSON 스키마는 `src/gps_field_ws_bridge/README.md`를 참고하세요.
