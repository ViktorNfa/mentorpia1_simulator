import os
from glob import glob
from setuptools import setup

package_name = 'mentorpia1_simulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.*'))),
        (os.path.join('share', package_name, 'meshes/acker'), glob(os.path.join('meshes/acker', '*.*'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Victor Nan Fernandez-Ayala',
    maintainer_email='viktornfa@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'ackermann_to_twist = mentorpia1_simulator.ackermann_to_twist:main',
        ],
    },
)
