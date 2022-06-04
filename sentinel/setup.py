from setuptools import setup
import glob

package_name = 'sentinel'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch')),
        ('share/' + package_name + '/config', glob.glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sofia Talancon',
    maintainer_email='sofia.talancon.fernandez@usi.ch',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'talker = sentinel.anomaly_publisher:main',
        'navigator = sentinel.navigator:main',
        'noise = sentinel.noise_estimation:main',
        'follow_path = sentinel.follow_path_action:main',
        'sentinel = sentinel.sentinel:main',
        'follower = sentinel.follower:main',
        'speed_controller = sentinel.speed_controller:main',
        'followSpeed_controller = sentinel.followSpeed_controller:main',
        'scenario_manager = sentinel.scenario_manager:main',
        ],
    },
)
