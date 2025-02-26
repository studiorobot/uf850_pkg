from setuptools import find_packages, setup

package_name = 'uf850_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        ],
    },
)
