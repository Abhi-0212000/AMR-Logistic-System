from setuptools import find_packages, setup

package_name = "amr_localization"

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
    description="Dummy localization node for AMR",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dummy_localization_node = amr_localization.dummy_localization_node:main",
        ],
    },
)
