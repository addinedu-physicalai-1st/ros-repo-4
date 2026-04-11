from setuptools import find_packages, setup

package_name = "gps_field_ws_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/ws_gps_bridge.launch.py"]),
        ("share/" + package_name + "/config", ["config/.env.example"]),
    ],
    install_requires=["setuptools", "websockets>=12"],
    zip_safe=True,
    maintainer="GPSTopicTest",
    maintainer_email="gpstest@example.com",
    description="WebSocket to PinkyGps ROS publisher bridge",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ws_gps_publisher = gps_field_ws_bridge.ws_gps_publisher:main",
            "ws_gps_client_demo = gps_field_ws_bridge.ws_gps_client_demo:main",
        ],
    },
)
