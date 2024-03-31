from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tank'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
           ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch/'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf/'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'rviz/'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'meshes/'), glob('meshes/*.STL')),
        (os.path.join('share', package_name, 'config/'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'msg/'), glob('msg/*.msg')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'minimal_publisher = tank.minimal_publisher:main',
            'ArUco = tank.ArUco:main',
            'reboot = tank.reboot:main',
            'move_base = tank.move_base:main',
            'angle_radius = tank.angle_radius:main',
            'screen = tank.screen:main',
            ],

    },
)
