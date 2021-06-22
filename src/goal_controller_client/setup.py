import os
from glob import glob
from setuptools import setup

package_name = 'goal_controller_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('qt_ui/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gonzalo Cervetti',
    maintainer_email='g.cervetti@ekumenlabs.com',
    description='Sends goals to goal_controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_controller_client = goal_controller_client.goal_controller_client:main',
        ],
    },
)
