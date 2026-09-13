from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'ai_robot_behaviors'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='AI Robot Team',
    maintainer_email='maintainer@example.com',
    description='Deterministic non-AI reactive behaviors for the mobile robot.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'reactive_avoidance = ai_robot_behaviors.reactive_avoidance_node:main',
        ],
    },
)
