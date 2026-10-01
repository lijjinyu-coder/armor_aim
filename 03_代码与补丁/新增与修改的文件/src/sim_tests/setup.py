from setuptools import find_packages, setup

package_name = "sim_tests"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Minimal ROS2 communication test programs for simulator integration.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "image_probe = sim_tests.image_probe:main",
            "gimbal_probe = sim_tests.gimbal_probe:main",
        ],
    },
)
