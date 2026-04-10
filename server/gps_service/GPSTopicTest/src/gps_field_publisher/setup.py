from setuptools import find_packages, setup

package_name = "gps_field_publisher"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/field_gps_publisher.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="GPSTopicTest",
    maintainer_email="gpstest@example.com",
    description="Publish simulated Pinky field GPS coordinates",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "field_gps_publisher = gps_field_publisher.publisher_node:main",
        ],
    },
)
