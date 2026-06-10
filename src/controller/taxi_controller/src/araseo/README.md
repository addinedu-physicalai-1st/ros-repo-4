# Araseo 시뮬레이션 구성

`araseo`는 Gazebo Sim에서 Pinky 로봇을 실행하고, ROS 2 토픽과 연결하는
패키지 묶음입니다.

## 핵심 실행 흐름

```text
simulation.launch.xml
  -> gz_world.launch.xml
  -> bringup_gz_taxi.launch.xml
  -> virtual_gps.launch.xml
```

## 주요 launch 파일

### `araseo_gz_sim/launch/simulation.launch.xml`

시뮬레이션의 최상위 launch 파일입니다.

- 월드 실행, 로봇 bringup, 가상 GPS launch 파일을 함께 실행합니다.
- 기본 로봇 ID는 `54`이며, namespace는 `pinky_54`가 됩니다.

### `araseo_gz_sim/launch/gz_world.launch.xml`

Gazebo Sim과 시뮬레이션 시간을 담당합니다.

- `minicity.sdf` 월드를 Gazebo Sim으로 실행합니다.
- Gazebo `clock`을 ROS 2 `/clock`으로 bridge합니다.

### `ros_gz_sim/launch/gz_sim.launch.py`

이 파일은 이 저장소 안에 있는 코드가 아니라 ROS Jazzy에 설치된 파일입니다.

```text
/opt/ros/jazzy/share/ros_gz_sim/launch/gz_sim.launch.py
```

역할은 Gazebo Sim을 실행하고 `minicity.sdf` 월드를 로드하는 것입니다.
즉, 월드를 새로 만드는 것이 아니라 이미 작성된 SDF 월드 파일을 실행합니다.

### `araseo_gz_sim/launch/bringup_gz_taxi.launch.xml`

실행 중인 Gazebo 월드에 Pinky 로봇을 추가하고 기본 ROS 인터페이스를 엽니다.

- `pinky_id`로 로봇 namespace와 이름을 만듭니다.
- `upload_robot.launch.py`를 통해 로봇 URDF/xacro description을 준비합니다.
- `ros_gz_sim create`로 Gazebo 월드에 로봇 entity를 생성합니다.
- odom, scan, cmd_vel, joint_states, TF, camera 관련 bridge를 실행합니다.

### `araseo_gz_sim/launch/virtual_gps.launch.xml`

Gazebo pose를 가상 GPS처럼 변환하고 odom을 보정합니다.

- Gazebo 모델 pose를 ROS `PoseStamped`로 bridge합니다.
- `gazebo_gps_publisher.py`로 `PinkyGps` 메시지를 발행합니다.
- `gps_odometry_calibrator`로 GPS와 odom 기반 `map -> odom` TF를 보정합니다.

### `pinky_description/launch/upload_robot.launch.py`

Pinky 로봇 모델 정보를 ROS에 올립니다.

- xacro 파일로 `robot_description`을 생성합니다.
- `robot_state_publisher`로 TF 정보를 발행합니다.
- `joint_state_publisher`를 실행합니다.

## 역할 정리

| 구성 요소 | 역할 |
| --- | --- |
| `simulation.launch.xml` | 전체 시뮬레이션 통합 실행 |
| `gz_world.launch.xml` | Gazebo Sim 실행, 월드 로드, `/clock` bridge |
| `bringup_gz_taxi.launch.xml` | 로봇 description, 스폰, 기본 bridge |
| `virtual_gps.launch.xml` | Gazebo pose 기반 가상 GPS, odom 보정 |
| `upload_robot.launch.py` | 로봇 description 및 TF 준비 |
| `ros_gz_bridge` | Gazebo 토픽과 ROS 2 토픽 연결 |
| `ros_gz_image` | Gazebo 카메라 이미지를 ROS 2 이미지 토픽으로 연결 |

`/clock` bridge는 책임 분리에 따라 `gz_world.launch.xml`에서만 실행합니다.

## 한 줄 요약

`simulation.launch.xml`은 Gazebo 월드를 켜고, Pinky 로봇을 스폰한 뒤,
기본 ROS 인터페이스와 가상 GPS를 함께 실행하는 통합 launch 파일입니다.
