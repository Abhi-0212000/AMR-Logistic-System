from setuptools import find_packages, setup

package_name = "amr_central_management"

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
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "amr_central_management = amr_central_management.central_management:main"
        ],
    },
)
