from setuptools import setup


package_name = 'debug_servop'


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
                'launch/debug_servop.launch.py',
            ],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maintainer',
    maintainer_email='maintainer@example.com',
    description='ServoP debug GUI with linear waypoint planning, RViz TF/Marker visualization, and TF-only preview mode.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_p_debug_gui = debug_servop.servo_p_debug_gui:main',
            # Backward-compatible executable name used by existing launch files/scripts.
            'debug_servop_node = debug_servop.servo_p_debug_gui:main',
        ],
    },
)
