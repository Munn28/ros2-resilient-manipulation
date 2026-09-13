import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'resilient_bringup'


setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        (
            'share/' + package_name,
            ['package.xml']
        ),

        (
            os.path.join(
                'share',
                package_name,
                'launch'
            ),
            glob('launch/*.launch.py')
        ),

        (
            os.path.join(
                'share',
                package_name,
                'worlds'
            ),
            glob('worlds/*.sdf')
        ),

        (
            os.path.join(
                'share',
                package_name,
                'config'
            ),
            glob('config/*.yaml')
        ),
    ],

    install_requires=[
        'setuptools'
    ],

    zip_safe=True,

    maintainer='munns',

    maintainer_email='munnsharawat1472@gmail.com',

    description=(
        'Bringup package for the resilient '
        'vision-guided manipulation cell.'
    ),

    license='MIT',

    tests_require=[
        'pytest'
    ],

    entry_points={
        'console_scripts': [],
    },
)
