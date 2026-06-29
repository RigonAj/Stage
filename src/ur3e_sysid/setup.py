from setuptools import find_packages, setup

package_name = "ur3e_sysid"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rigon",
    maintainer_email="rigon@example.com",
    description="UR3e actuator system identification: online sweep + offline gain/latency/friction fit.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "run_sweep = ur3e_sysid.run_sweep:main",
            "fit_gains = ur3e_sysid.fit_gains:main",
        ],
    },
)
