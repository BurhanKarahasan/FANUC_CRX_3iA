from setuptools import find_packages, setup

package_name = "fanuc_custom_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/crx3ia_collision_scene.launch.py"]),
        ("share/" + package_name + "/assets", [
            "assets/KUBTEC_workspace_10cm.stl",
            "assets/KUBTEC_workspace_20cm.stl",
            "assets/stepper_motor.stl",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "collision_scene_publisher = fanuc_custom_tools.collision_scene_publisher:main",
            "workspace_boundary_sweep = fanuc_custom_tools.workspace_boundary_sweep:main",
        ],
    },
)