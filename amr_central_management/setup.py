# Copyright 2025 Abhishek Nannuri
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
from setuptools import find_packages, setup

package_name = "amr_central_management"

if len(sys.argv) >= 2 and sys.argv[1] != "clean":
    from generate_parameter_library_py.setup_helper import generate_parameter_module

    # set module_name and yaml file
    module_name = "amr_central_management_params"
    yaml_file = "amr_central_management/amr_central_management_params.yaml"
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
    description="amr_central_management: It manages the AMR goals, error handling",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [f"amr_central_management_node = {package_name}.central_management:main"],
    },
)
