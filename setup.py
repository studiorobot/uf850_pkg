from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'uf850_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='homandat',
    maintainer_email='homandat2002@gmail.com',
    description='Package for controlling a 6-DOF UF850 robot arm with custom messages',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uf850_node = uf850_pkg.uf850_node:main',
            'joy2uf850_node = uf850_pkg.joy2uf850_node:main',
            'state_machine_node = uf850_pkg.state_machine_node:main',
            'voice_node = voice_pkg.transcribe:main',
        ],
    },
)
