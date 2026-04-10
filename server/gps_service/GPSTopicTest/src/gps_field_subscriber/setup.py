from setuptools import find_packages, setup

package_name = "gps_field_subscriber"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "requirements-oled.txt"]),
        ("share/" + package_name + "/launch", ["launch/field_gps_subscriber.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="GPSTopicTest",
    maintainer_email="gpstest@example.com",
    description="Subscribe Pinky field GPS and show on LCD window",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "field_gps_subscriber = gps_field_subscriber.subscriber_node:main",
            "field_oled_selftest = gps_field_subscriber.oled_selftest:main",
        ],
    },
)
