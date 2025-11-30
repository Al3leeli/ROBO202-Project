from setuptools import setup
import os
from glob import glob

package_name = 'tricycle_cam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # URDF files (your diff_robot.urdf + obstacle.urdf)
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        # launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@university.edu',
    description='Tricycle robot with camera simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_subscriber = tricycle_cam.image_subscriber:main',
        ],
    },
)
