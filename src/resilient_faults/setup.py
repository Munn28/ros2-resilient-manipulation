from setuptools import find_packages, setup

package_name = 'resilient_faults'

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
    description='Fault injection framework for resilient robotic manipulation',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
	     'fault_injector = resilient_faults.fault_injector:main',
	     'gripper_fault_proxy = resilient_faults.gripper_fault_proxy:main',
	     'path_blocker = resilient_faults.path_blocker:main',
	     'fault_event_logger = resilient_faults.fault_event_logger:main',
        ],
    },
)
