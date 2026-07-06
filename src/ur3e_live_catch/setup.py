from glob import glob

from setuptools import find_packages, setup

package_name = "ur3e_live_catch"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rigon",
    maintainer_email="rigon@example.com",
    description="UR3e live closed-loop ball catch: ball-frame, observation, policy runtime, test source.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "test_ball_node = ur3e_live_catch.test_ball_node:main",
            "float32_adapter = ur3e_live_catch.float32_adapter:main",
            "live_catch_node = ur3e_live_catch.live_catch_node:main",
            "latency_report = ur3e_live_catch.latency_report:main",
            "ball_regression_node = ur3e_live_catch.ball_regression_node:main",
        ],
    },
)
