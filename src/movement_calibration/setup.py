from setuptools import setup


package_name = 'movement_calibration'


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
                'launch/movement_calibration.launch.py',
                'launch/movement_calibration_gui.launch.py',
            ],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maintainer',
    maintainer_email='maintainer@example.com',
    description='Live movement calibration node for RelMovL speed characterization.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'movement_calibration = movement_calibration.movement_calibration:main',
            'movement_calibration_gui = movement_calibration.movement_calibration_gui:main',
        ],
    },
)
