from setuptools import setup
from glob import glob

package_name = 'tars_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # directory tars_sim/ with __init__.py
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='Gazebo-ready TARS (Interstellar) URDF with ros2_control.',
    license='MIT',
    entry_points={'console_scripts': []},
)
