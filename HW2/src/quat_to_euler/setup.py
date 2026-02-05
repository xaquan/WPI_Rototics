from setuptools import find_packages, setup

package_name = 'quat_to_euler'

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
    maintainer='xquan',
    maintainer_email='xquan1@wpi.edu',
    description='TODO: receive quaternion and convert to euler angles',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [            
                'quat_to_euler_node = quat_to_euler.quat_to_euler_node:main',
        ],
    },
)
