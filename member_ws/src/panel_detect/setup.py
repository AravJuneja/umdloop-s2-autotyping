"""Package definition for panel_detect."""

from glob import glob

from setuptools import find_packages, setup

package_name = 'panel_detect'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Victor Casado',
    maintainer_email='victor.casado.nyc@gmail.com',
    description="Find the panel's ArUco markers in the camera image.",
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'panel_detect = panel_detect.panel_detect:main',
            'key_projector = panel_detect.key_positions:main'
        ],
    },
)
