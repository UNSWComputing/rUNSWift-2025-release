import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'isaacsim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='reilly',
    maintainer_email='reillyfox6@gmail.com',
    description='Package for simulation helper nodes',
    license='See License in license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'isaacsim_bridge = isaacsim.isaacsim_bridge:main'
        ],
    },
)
