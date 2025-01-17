from setuptools import find_packages, setup

package_name = 'amr_local_planner_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abhi',
    maintainer_email='nannuriabhi2000@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulated_localization = amr_local_planner_py.simulatedLocalization:main',
            'local_planner = amr_local_planner_py.localPlannerNode:main'
        ],
    },
)
