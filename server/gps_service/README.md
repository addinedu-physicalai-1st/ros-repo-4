# GPSTopicTest (ROS 2)

필드 크기 **1880 mm x 1410 mm** 를 가정한 “GPS” 형태 좌표(`x_mm`, `y_mm`)를 기본 토픽 **`/pinky_{pinky_id}/gps_pos`** 로 **발행**하고, 구독 쪽에서 **Tk / SPI LCD** 등으로 보여 주는 예제입니다. (`src/gps_field_ws_bridge`: 웹소켓 → 동일 토픽)

## 폴더 안내

| 경로 | 설명 |
|------|------|
| `src/gps_field_msgs/` | 메시지 정의 `gps_field_msgs/PinkyGps` |
| `src/gps_field_publisher/` | 발행 노드 `field_gps_publisher` |
| `src/gps_field_subscriber/` | 구독 + 창 표시 `field_gps_subscriber` |
| `src/gps_field_ws_bridge/` | 웹소켓 → `PinkyGps` 발행 |
| **`publisher/README.md`** | **발행(서버)** 설명 — **한글** |
| **`subscriber/README.md`** | **구독(핑키)** 설명 — **한글** |

## 빌드

`<distro>` 에는 `jazzy`, `humble` 등 실제 폴더 이름을 넣습니다 (`/opt/ros/<distro>/`).

```bash
source /opt/ros/<distro>/setup.bash
cd ~/GPSTopicTest
colcon build --symlink-install
source install/setup.bash
```

## ROS_DOMAIN_ID (중요)

서버(발행)와 핑키(구독)는 **같은 숫자**를 써야 같은 ROS 2 네트워크에서 만납니다. (이 문서 예시는 핑키 기준 **도메인 ID 23** 을 사용합니다.)

```bash
export ROS_DOMAIN_ID=23
```

실행 예·파라미터·문제 해결은 **`publisher/README.md`**, **`subscriber/README.md`** 를 읽으세요.

## Git 및 호스트 PC ↔ 핑키 동기화

이 워크스페이스는 **`build/` · `install/` · `log/` 는 제외**하고 Git으로 추적합니다. (초기 커밋은 핑키 쪽 작업 기준 스냅샷.)

**호스트와 핑키(예: `pinky@192.168.25.61`)에 같은 소스를 맞추려면:**

1. **한쪽을 기준으로** `git pull` / `git push` (원격 저장소가 있으면) 또는
2. **rsync 예시** (핑키 → 호스트, 소스만):

```bash
rsync -av --delete \
  pinky@192.168.25.61:~/GPSTopicTest/src/ \
  ./GPSTopicTest/src/
rsync -av pinky@192.168.25.61:~/GPSTopicTest/publisher/ ./GPSTopicTest/publisher/
rsync -av pinky@192.168.25.61:~/GPSTopicTest/subscriber/ ./GPSTopicTest/subscriber/
rsync -av pinky@192.168.25.61:~/GPSTopicTest/README.md ./GPSTopicTest/
```

반대(호스트 → 핑키)는 경로만 바꿉니다. 전체 트리를 덮을 때는 `--exclude 'build' --exclude 'install' --exclude 'log'` 를 추가하세요.

**차이만 보려면:** 한쪽에서 `git diff`, 또는 `diff -ru 핑키복사본/GPSTopicTest/src 호스트/GPSTopicTest/src`.
