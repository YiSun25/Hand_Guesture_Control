from setuptools import find_packages, setup

package_name = 'hand_gesture_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gesture_control_dual_ur5.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yi Sun',
    maintainer_email='yi.sun@rwth-aachen.de',
    description='Unified hand gesture publisher with custom ArmCommand message',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_gesture_publisher = hand_gesture_control.hand_gesture_publisher:main',
            'hand_gesture_subscriber = hand_gesture_control.hand_gesture_subscriber:main',
        ],
    },
)
