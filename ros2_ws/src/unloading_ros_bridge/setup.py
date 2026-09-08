from setuptools import find_packages, setup

package_name = "unloading_ros_bridge"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "unloading-contracts==1.0.0"],
    zip_safe=True,
    maintainer="Unloading Simulation Maintainers",
    maintainer_email="maintainers@example.invalid",
    description="ROS 2 Humble bridge for versioned unloading contracts",
    license="MIT",
    entry_points={"console_scripts": [
        "perception_node = unloading_ros_bridge.perception_node:main",
        "world_bridge_node = unloading_ros_bridge.world_bridge_node:main",
        "execution_bridge_node = unloading_ros_bridge.execution_bridge_node:main",
        "mock_follow_joint_trajectory = unloading_ros_bridge.mock_follow_joint_trajectory:main",
    ]},
)
