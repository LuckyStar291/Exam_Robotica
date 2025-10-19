import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='isaac',
    maintainer_email='isaac@todo.todo',
    description='Package for sensor publishers, filter, and display nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "sensor1 = py_pubsub.sensor1:main",
            "sensor2 = py_pubsub.sensor2:main",
            "sensor3 = py_pubsub.sensor3:main",
            "filter = py_pubsub.filter:main",
            "display = py_pubsub.display:main",
        ],
    },
)
