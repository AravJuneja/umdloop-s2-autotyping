from setuptools import find_packages, setup

package_name = 'arm_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arav',
    maintainer_email='aravjuneja@gmail.com',
    description='Drive the arm to a joint-space pose, closed loop, within its limits.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arm_control = arm_control.arm_control:main'
        ],
    },
)
