from setuptools import find_packages, setup


package_name = "teleop_hand_telemetry"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/receiver.launch.py"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Pico arm-hand teleop maintainers",
    maintainer_email="maintainers@example.com",
    description="Read-only UDP-to-ROS Wuji hand telemetry receiver.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "hand_telemetry_receiver = teleop_hand_telemetry.receiver:main",
        ]
    },
)
