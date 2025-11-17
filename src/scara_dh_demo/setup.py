from setuptools import setup

package_name = 'scara_dh_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/dh_motion.launch.py',
            'launch/traj_demo.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mariaantonia',
    maintainer_email='maria@example.com',
    description='Demo de cinemática DH para robô SCARA com ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dh_motion_demo = scara_dh_demo.dh_motion_demo:main',
            'scara_traj_demo = scara_dh_demo.scara_traj_demo:main',
        ],
    },
)

