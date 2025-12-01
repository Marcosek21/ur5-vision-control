from setuptools import setup

package_name = 'ur5_vision_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ur5_vision_control.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Vision control of UR5 using camera clicks and ArUco markers.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'click_controller_node = ur5_vision_control.click_controller_node:main',
            'aruco_controller_node = ur5_vision_control.aruco_controller_node:main',
        ],
    },
)