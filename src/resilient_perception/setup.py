from setuptools import find_packages, setup

package_name = 'resilient_perception'

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
    maintainer='munns',
    maintainer_email='munnsharawat1472@gmail.com',
    description='MIT',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'red_object_detector = resilient_perception.red_object_detector:main',
		'object_pose_transformer = resilient_perception.object_pose_transformer:main',
        ],
    },
)
