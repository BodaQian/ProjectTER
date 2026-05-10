import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'qbd_insight'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'yolo'), glob('yolo/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='qbd',
    maintainer_email='qianboda1234@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard = qbd_insight.offboard:main',
            'decode_px4_fmu_out_vehicle_status = qbd_insight.msg_px4_fmu_out_vehicle_status:main',
            'keyboard_position = qbd_insight.keyboard_position:main',
            'move_position = qbd_insight.move_position:main',
            'keyboard_velocity = qbd_insight.keyboard_velocity:main',
            'move_velocity = qbd_insight.move_velocity:main',
            'multi_move_velocity = qbd_insight.multi_move_velocity:main',
            'octomap_dynamic = qbd_insight.octomap_dynamic:main',
            'yolo_node = qbd_insight.yolo_node:main',
            'track = qbd_insight.track:main',
        ],
    },
)
