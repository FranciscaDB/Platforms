from setuptools import setup

package_name = 'jetros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_jetros.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetros',
    maintainer_email='jetros@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller_node = jetros.motor_controller_node:main',
            'encoder_reader_left_node = jetros.encoder_reader_left_node:main',
            'encoder_reader_right_node = jetros.encoder_reader_right_node:main',
            'odom_publisher_node = jetros.odometry_node:main',
            'action_prioritizer_node = jetros.action_prioritizer_node:main',
            'pid_node = jetros.PID_node:main',
            'steering_node = jetros.steering_node:main',
            "filter = jetros.filter:main",
        ],
    },
)
