from setuptools import setup, find_packages
import os
from glob import glob

package_name = 's1_attack_simulation'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # Include world files
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UAV Security Research',
    maintainer_email='user@example.com',
    description='S1 Normative Poisoning Attack Simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uav_agent_node = s1_attack_simulation.uav_agent_node:main',
            'mission_executor = s1_attack_simulation.mission_executor:main',
            'metrics_collector = s1_attack_simulation.metrics_collector:main',
            'run_baseline = s1_attack_simulation.run_baseline_ros:main',
            'run_injection = s1_attack_simulation.run_injection_ros:main',
            'run_test = s1_attack_simulation.run_test_ros:main',
        ],
    },
)
