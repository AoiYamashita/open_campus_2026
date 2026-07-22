from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_arm_controll'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share",package_name),glob("launch/*.launch.py"))
    ],
    install_requires=[
        'setuptools',
        'Arm_lib',
        'smbus',
        'tkinter',
        'opencv-python',
        "ultralytics",
        ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='yamashitaaoi1230@icloud.com',
    description='dofbot controll pkg',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "controll = robot_arm_controll.controll:main",
            "app = robot_arm_controll.controll_app:main",
            "cam_process = robot_arm_controll.cam_process:main",
            "yolo = robot_arm_controll.yolo_process:main"
        ],
    },
)
