from setuptools import setup


package_name = 'robot_cell_orchestrator'


setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/launch',
            [
                'launch/robot_cell_orchestrator.launch.py',
                'launch/robot_runtime_headless.launch.py',
            ],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maintainer',
    maintainer_email='maintainer@example.com',
    description='Robot cell orchestrator GUI and online API for coordinating production pick/place flow.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_cell_orchestrator_gui = robot_cell_orchestrator.robot_cell_orchestrator_gui:main',
        ],
    },
)
