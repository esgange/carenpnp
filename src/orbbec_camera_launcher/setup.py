from setuptools import setup


package_name = 'orbbec_camera_launcher'


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
                'launch/camera_launcher.launch.py',
                'launch/camera_headless.launch.py',
            ],
        ),
        (
            'lib/' + package_name,
            [
                'scripts/camera_launcher_gui',
            ],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maintainer',
    maintainer_email='maintainer@example.com',
    description='Operator GUI for launching two Orbbec cameras by serial number.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
)
