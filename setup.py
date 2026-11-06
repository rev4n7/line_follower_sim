from glob import glob

from setuptools import setup

package_name = 'line_follower_sim'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rev4n7',
    description='Camera line follower with pure pursuit in Gazebo Harmonic',
    license='MIT',
    entry_points={
        'console_scripts': [
            'follower = line_follower_sim.follower:main',
        ],
    },
)
