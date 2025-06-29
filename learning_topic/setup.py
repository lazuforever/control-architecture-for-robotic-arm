from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'learning_topic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # IMPORTANTE: Añadir esta línea para incluir archivos de launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lunanirvana',
    maintainer_email='lunanirvana@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # CORREGIDO: Eliminados espacios extra en los nombres
            'topic_helloworld_pub = learning_topic.topic_helloworld_pub:main',
            'topic_helloworld_sub = learning_topic.topic_helloworld_sub:main',
            'topic_webcam_pub = learning_topic.topic_webcam_pub:main',
            'finger_detector = learning_topic.finger_detector:main',
            'topic_webcam_sub = learning_topic.topic_webcam_sub:main',
            'detect_object_table = learning_topic.detect_object_table:main',
            'interface_object_pub = learning_topic.interface_object_pub:main',
            'interface_object_sub = learning_topic.interface_object_sub:main',
            'trayectoria = learning_topic.trayectoria:main',
        ],
    },
)