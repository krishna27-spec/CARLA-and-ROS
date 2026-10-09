from glob import glob

from setuptools import find_packages, setup

package_name = 'sim_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.csv')),
        ('share/' + package_name + '/models/ackermann_car',
            glob('models/ackermann_car/*')),
        ('share/' + package_name + '/models/traffic_cone',
            glob('models/traffic_cone/*.*')),
        ('share/' + package_name + '/models/traffic_cone/meshes',
            glob('models/traffic_cone/meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='srikrishna',
    maintainer_email='you@example.com',
    description='Gazebo bringup for the controls_pkg controllers',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
)
