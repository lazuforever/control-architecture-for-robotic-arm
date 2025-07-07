from setuptools import setup

package_name = 'visualizacion_angulos'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TU_NOMBRE',
    maintainer_email='tu@email.com',
    description='Nodo ROS 2 que grafica ángulos de /serial_joint_states y /final_angles',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'angle_plotter = visualizacion_angulos.plotter_node:main',
        ],
    },
)
