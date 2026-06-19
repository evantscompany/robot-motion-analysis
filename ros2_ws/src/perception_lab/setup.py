from setuptools import find_packages, setup
from glob import glob

package_name = 'perception_lab'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         glob('launch/*.py'))
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
            'fake_camera = perception_lab.fake_camera:main',
            'image_viewer = perception_lab.image_viewer:main',
            'red_ball_detector = perception_lab.red_ball_detector:main',
            'p_controller_tracker = perception_lab.p_controller_tracker:main',
            'visual_servo_simulator = perception_lab.visual_servo_simulator:main',
            'fake_depth_camera = perception_lab.fake_depth_camera:main',
            'object_localizer = perception_lab.object_localizer:main'

        ],
    },
)
