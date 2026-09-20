"""Package definition for robot_state."""

from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_state'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Victor Casado',
    maintainer_email='victor.casado.nyc@gmail.com',
    description='Normalize simulator inputs into timestamped robot observations.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': ['robot_state = robot_state.robot_state:main'],
    },
)
