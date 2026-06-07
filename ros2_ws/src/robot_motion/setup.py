from setuptools import find_packages, setup

package_name = 'robot_motion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='msm031',
    maintainer_email='evan.tscompany@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		    'fake_robot_simulator = robot_motion.fake_robot_simulator:main',

		    'pure_pursuit_tracker = robot_motion.pure_pursuit_tracker:main',

            'trajectory_visualizer = robot_motion.trajectory_visualizer:main',

            'robot_marker = robot_motion.robot_marker:main',

            'cross_track_visualizer = robot_motion.cross_track_visualizer:main',

            'segment_cte_visualizer = robot_motion.segment_cte_visualizer:main'

        ],
    },
)
