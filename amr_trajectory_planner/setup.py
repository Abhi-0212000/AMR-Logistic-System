import sys
from setuptools import find_packages, setup

package_name = "amr_trajectory_planner"

if len(sys.argv) >= 2 and sys.argv[1] != "clean":
    from generate_parameter_library_py.setup_helper import generate_parameter_module

    # set module_name and yaml file
    module_name = "amr_trajectory_planner_params"
    yaml_file = "amr_trajectory_planner/amr_trajectory_planner_params.yaml"
    # validation_module = 'generate_parameter_module_example.custom_validation'
    generate_parameter_module(module_name, yaml_file)

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Abhishek Nannuri",
    maintainer_email="abhishek.nannuri@outlook.com",
    description="amr_trajectory_planner: AMR Trajectory Planner",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"amr_trajectory_planner_node = {package_name}.amr_trajectory_planner_node:main",
            f"amr_trajectory_planner_test = {package_name}.amr_traffic_rules:main",
        ],
    },
)
