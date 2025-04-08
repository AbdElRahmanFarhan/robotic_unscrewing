from setuptools import find_packages, setup
import os
import glob


package_name = 'lion_robot_description'
irb6640_205_files = glob.glob('meshes/irb6640_205/*.dae')
screwdriver_files = glob.glob('meshes/screwdriver/*.dae')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/lion_robot_description/launch', ['launch/view_robot_launch.py']),
        ('share/lion_robot_description/urdf', ['urdf/robot_w_screwdriver.urdf']),
        ('share/lion_robot_description/meshes/irb6640_205', irb6640_205_files),
        ('share/lion_robot_description/meshes/screwdriver', screwdriver_files),
        ('share/lion_robot_description/config', ['config/robot.rviz']),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='abdofarhan75@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
