from setuptools import find_packages, setup

package_name = 'typist'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/typist.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arav',
    maintainer_email='aravjuneja@gmail.com',
    description='Type the launch key: key positions to joint goals to presses.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'typist = typist.typist:main',
            'e2e_check = typist.e2e_check:main',
        ],
    },
)
