from setuptools import find_packages, setup


package_name = "teleop_data_collector"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", ["launch/collector.launch.py"]),
    ],
    install_requires=["setuptools", "numpy", "pyarrow"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Pico arm-hand teleop maintainers",
    maintainer_email="maintainers@example.com",
    description="Read-only episode rosbag recorder and manual LeRobot v2 converter.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "rosbag_data_collector = teleop_data_collector.rosbag_recording_node:main",
            "revalidate_rosbag = teleop_data_collector.revalidate_bag:main",
            "rosbag_to_lerobot = teleop_data_collector.rosbag_to_lerobot:main",
        ]
    },
)
