import os
from glob import glob
from setuptools import find_packages, setup

package_name = "behaviours"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.*'))),
    ],
    include_package_data=True,
    package_data={
        "": ["datamodels/**/*", "skills/**/*", "utils/*"],
    },
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rUNSWift",
    maintainer_email="unswrobocup@gmail.com",
    description="rUNSWift behaviour package (Python)",
    license="SEE LICENSE IN LICENSE",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["blackboard = src.Blackboard:main", "node = src.BehaviourNode:main"],
    },
)
