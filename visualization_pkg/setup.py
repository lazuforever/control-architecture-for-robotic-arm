import os
from glob import glob 
from setuptools import find_packages, setup

package_name = 'visualization_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Include all launch files.
        (os.path.join('share', package_name, 'Launch'), glob(os.path.join('Launch', '*')))
    
    
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lunanirvana',
    maintainer_email='lunanirvana@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'publish_point = visualization_pkg.publish_point:main',
        ],
    },
)
