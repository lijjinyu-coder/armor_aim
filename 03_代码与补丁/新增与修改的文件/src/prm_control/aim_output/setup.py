from setuptools import find_packages, setup

package_name = "aim_output"

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
    description="Output abstraction: AimResult -> debug output and ROS2 gimbal commands.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "aim_output_node = aim_output.aim_output_node:main",
        ],
    },
)
