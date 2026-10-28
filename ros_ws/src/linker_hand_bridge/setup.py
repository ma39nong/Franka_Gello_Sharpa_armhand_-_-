from glob import glob

from setuptools import find_packages, setup


setup(
    name="linker_hand_bridge",
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/linker_hand_bridge"]),
        ("share/linker_hand_bridge", ["package.xml"]),
        ("share/linker_hand_bridge/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="descfly",
    maintainer_email="descfly@example.com",
    description="Safety-gated, per-side-profiled LinkerHand command bridge.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "bridge = linker_hand_bridge.node:main",
            "slot_probe = linker_hand_bridge.slot_probe:main",
        ]
    },
)
