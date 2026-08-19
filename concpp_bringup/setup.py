import os
from glob import glob
from setuptools import setup

package_name = 'concpp_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.map')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tahseen',
    maintainer_email='tahseen@todo.todo',
    description='Bringup: launch files, maps, RViz config for ConCPP',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)