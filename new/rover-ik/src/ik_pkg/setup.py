from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'ik_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
  	(os.path.join('share', package_name), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tristan',
    maintainer_email='tristanmcginnis26@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = ik_pkg.state_publisher:main'
        ],
    },
)
