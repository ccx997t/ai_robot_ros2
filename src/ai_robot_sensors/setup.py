from setuptools import find_packages, setup

package_name = 'ai_robot_sensors'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='AI Robot Team',
    maintainer_email='maintainer@example.com',
    description='Simulation-to-public sensor contract adapters and diagnostics.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'sensor_adapter = ai_robot_sensors.sensor_adapter:main',
        'image_processor = ai_robot_sensors.image_processor:main',
        'fault_injector = ai_robot_sensors.fault_injector:main',
    ]},
)
