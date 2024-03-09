from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gzb_sim'

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
        (os.path.join('share', package_name, 'world/'), glob('world/*.sdf')),
        (os.path.join('share', package_name, 'meshes/'), glob('meshes/*.STL')),
        (os.path.join('share', package_name, 'world/Arena/meshes/'), glob('world/Arena/meshes/*.dae')),
        (os.path.join('share', package_name, 'world/Arena/meshes/'), glob('world/Arena/meshes/*.png')),
        (os.path.join('share', package_name, 'world/plant/meshes/'), glob('world/plant/meshes/*.png')),
        (os.path.join('share', package_name, 'world/plant/meshes/'), glob('world/plant/meshes/*.dae')),
        (os.path.join('share', package_name, 'world/pot/meshes/'), glob('world/pot/meshes/*.dae')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='TManshin@yandex.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
