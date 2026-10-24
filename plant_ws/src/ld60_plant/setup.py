import os
from glob import glob

from setuptools import setup

package_name = 'ld60_plant'


def tree(directory):
    """Install every file under directory, keeping its sub-folders."""
    out = {}
    for path in glob(os.path.join(directory, '**', '*'), recursive=True):
        if os.path.isfile(path):
            dest = os.path.join('share', package_name, os.path.dirname(path))
            out.setdefault(dest, []).append(path)
    return list(out.items())


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *tree('launch'),
        *tree('config'),
        *tree('models'),
        *tree('urdf'),
        *tree('worlds'),
        *tree('web'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ahmad Zainul Arifin',
    maintainer_email='Keyzoo0@users.noreply.github.com',
    description='Omron LD-60 palletizing plant simulation with a MobilePlanner-style dashboard',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'amr_manager = ld60_plant.amr_manager:main',
            'web_server = ld60_plant.web_server:main',
            'plant_manager = ld60_plant.plant_manager:main',
            'plant_check = ld60_plant.plant_check:main',
        ],
    },
)
