from glob import glob
from setuptools import setup

package_name = 'dvrk_python'

setup(
    name = package_name,
    version = '2.2.1',
    packages = ['dvrk'],
    package_dir = {'': 'src'},
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, glob('scripts/*.py')),
    ],
    install_requires = ['setuptools'],
    zip_safe = True,
    maintainer = 'Anton Deguet',
    maintainer_email = 'anton.deguet@jhu.edu',
    description = 'dVRK Python client for ROS2',
    license = 'MIT',
    entry_points = {
        'console_scripts': [
        ],
    },
)
