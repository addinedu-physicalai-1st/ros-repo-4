cmake_minimum_required(VERSION 3.8)
project(araseo_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

set(CMAKE_CXX_STANDARD 14)

# OpenCV 패키지 찾기 전에 CUDA 버전 미스매치로 인한 에러 우회 설정 추가
set(CUDA_FOUND TRUE CACHE INTERNAL "")
set(CUDA_VERSION "13.1" CACHE INTERNAL "")
set(CUDA_INCLUDE_DIRS "" CACHE INTERNAL "")
set(CUDA_LIBRARIES "" CACHE INTERNAL "")
set(CMAKE_POLICY_DEFAULT_CMP0146 OLD) # FindCUDA 관련 경고 억제

# OpenCV 패키지 찾기
find_package(OpenCV REQUIRED)

# 플래폼 옵션 추가 (기본값 PC)
if(NOT BUILD_TARGET)
    set(BUILD_TARGET "PC")
endif()

if(BUILD_TARGET STREQUAL "PC")
    add_definitions(-DPLATFORM_PC)
    message(STATUS "Build target is PC")
elseif(BUILD_TARGET STREQUAL "RASPI")
    add_definitions(-DPLATFORM_RASPI)
    message(STATUS "Build target is RASPI")
else()
    message(FATAL_ERROR "Unknown BUILD_TARGET: ${BUILD_TARGET}. Use PC or RASPI.")
endif()

# 헤더 경로 추가
include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/include
    ${OpenCV_INCLUDE_DIRS}
    /opt/ros/jazzy/include
    /opt/ros/jazzy/include/fastrtps
    /opt/ros/jazzy/include/fastcdr
    /usr/include
)

set(DXL_SOURCES)
set(LIDAR_SDK_SRC)
if(BUILD_TARGET STREQUAL "RASPI")
    if(NOT DXL_SDK_DIR)
        set(DXL_SDK_DIR /home/robo/projects/ros-project/DynamixelSDK/c++)
    endif()
    if(NOT LIDAR_SDK_DIR)
        set(LIDAR_SDK_DIR /home/pinky/pinky_pro/src/sllidar_ros2/sdk)
    endif()

    include_directories(
        ${DXL_SDK_DIR}/include
        ${DXL_SDK_DIR}/include/dynamixel_sdk
        ${LIDAR_SDK_DIR}/include
        ${LIDAR_SDK_DIR}/src
    )

    set(DXL_SOURCES
        ${DXL_SDK_DIR}/src/dynamixel_sdk/port_handler.cpp
        ${DXL_SDK_DIR}/src/dynamixel_sdk/port_handler_linux.cpp
        ${DXL_SDK_DIR}/src/dynamixel_sdk/packet_handler.cpp
        ${DXL_SDK_DIR}/src/dynamixel_sdk/protocol1_packet_handler.cpp
        ${DXL_SDK_DIR}/src/dynamixel_sdk/protocol2_packet_handler.cpp
    )

    file(GLOB LIDAR_SDK_SRC 
        "${LIDAR_SDK_DIR}/src/arch/linux/*.cpp"
        "${LIDAR_SDK_DIR}/src/hal/*.cpp"
        "${LIDAR_SDK_DIR}/src/*.cpp"
        "${LIDAR_SDK_DIR}/src/dataunpacker/*.cpp"
        "${LIDAR_SDK_DIR}/src/dataunpacker/unpacker/*.cpp"
    )

    if(NOT EXISTS ${DXL_SDK_DIR} OR NOT EXISTS ${LIDAR_SDK_DIR})
        message(WARNING "RASPI build selected but DXL/LIDAR SDK paths were not found.")
    endif()
endif()

message(STATUS "LANE_CPP target ENABLED")
add_executable(lane_cpp 
    src/main.cpp 
    src/Motor.cpp
    src/Lidar.cpp
    src/IMU.cpp
    src/CameraPublisher.cpp
    src/ADCSensor.cpp
    src/SensorDataSubscriber.cxx
    src/SensorDataPublisher.cxx
    src/SensorData.cxx
    src/SensorDataPubSubTypes.cxx
    ${DXL_SOURCES}
    ${LIDAR_SDK_SRC}
)

# PC용 조이스틱 퍼블리셔 (Sony T60 전용)
add_executable(joy_pub
    src/JoyPublisher.cpp
    src/JoystickManager.cpp
    src/SensorDataPublisher.cxx
    src/SensorData.cxx
    src/SensorDataPubSubTypes.cxx
)

# 웹 대시보드 브릿지 (DDS -> JSON/Base64)
add_executable(data_bridge
    src/web_dashboard/DataBridge.cpp
    src/SensorData.cxx
    src/SensorDataPubSubTypes.cxx
)

# DDS 모니터링 및 테스트 도구
add_executable(SensorDataApp 
    src/SensorDataPubSubMain.cxx 
    src/SensorDataPublisher.cxx
    src/SensorDataSubscriber.cxx
    src/SensorData.cxx
    src/SensorDataPubSubTypes.cxx
)

# ChArUco 렌즈 왜곡 캘리브레이터 (웹 대시보드 스트리밍 포함)
add_executable(camera_calibrator 
    src/camera_calibrator.cpp
    src/CameraPublisher.cpp
    src/SensorDataSubscriber.cxx
    src/SensorData.cxx
    src/SensorDataPubSubTypes.cxx
)

# 라이브러리 경로 설정
set(FASTDDS_LIBS /opt/ros/jazzy/lib/libfastrtps.so)
set(FASTCDR_LIBS /opt/ros/jazzy/lib/libfastcdr.so)

set(COMMON_LIBS ${OpenCV_LIBS} ${FASTDDS_LIBS} ${FASTCDR_LIBS} pthread)
if(BUILD_TARGET STREQUAL "RASPI")
    set(COMMON_LIBS ${COMMON_LIBS} rt)
endif()

target_link_libraries(lane_cpp ${COMMON_LIBS})
target_link_libraries(joy_pub ${COMMON_LIBS})
target_link_libraries(data_bridge ${COMMON_LIBS})
target_link_libraries(SensorDataApp ${COMMON_LIBS})
target_link_libraries(camera_calibrator ${COMMON_LIBS})

# Install executables so colcon can find them
install(TARGETS lane_cpp DESTINATION lib/${PROJECT_NAME})
install(TARGETS joy_pub data_bridge SensorDataApp camera_calibrator DESTINATION lib/${PROJECT_NAME})

# Install configuration/documentation files
install(FILES SensorData.idl VehicleControl.idl battery.py Navigation_Guide.md DESTINATION share/${PROJECT_NAME})

ament_package()
