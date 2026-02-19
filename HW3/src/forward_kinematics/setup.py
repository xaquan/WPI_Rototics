from setuptools import find_packages, setup

package_name = 'forward_kinematics'

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
    description='Forward kinematics package for robotic arm',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [            
                'fw_solver = forward_kinematics.subcriber_fw_solver:main'
        ],
    },
)
