from setuptools import find_packages, setup
from glob import glob

package_name = 'controls_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
        'share/' + package_name + '/models',
        glob('controls_pkg/*.pt')
        ),
    ],
    install_requires=['setuptools'],

    zip_safe=True,
    maintainer='srikrishna',
    maintainer_email='you@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "path_publisher = controls_pkg.path_publisher:main",
            'goal_path_publisher = controls_pkg.goal_path_publisher:main',
            'cmd_vel_relay = controls_pkg.cmd_vel_relay:main',
            'p_controller = controls_pkg.p_controller:main',
            'pd_controller = controls_pkg.pd_controller:main',
            'pi_controller = controls_pkg.pi_controller:main',
            'pid_controller = controls_pkg.pid_controller:main',
            'pure_pursuit_controller = controls_pkg.pure_pursuit_controller:main',
            'stanley_controller = controls_pkg.stanley_controller:main',
            'path_error_logger = controls_pkg.path_error_logger:main',
            'car_pure_pursuit = controls_pkg.car_pure_pursuit:main',
            'track_path_publisher = controls_pkg.track_path_publisher:main',
            'rl_controller = controls_pkg.rl_controller:main',
            "mpc = controls_pkg.mpc:main",
        ],
    },
)
