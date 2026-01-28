from setuptools import find_packages, setup

package_name = 'hw1_pubsub'

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
        description="Publish increasing integer numbers every one second, and subscribe to receive these numbers and print whether they are even or odd.",
        license='Apache-2.0',
        extras_require={
            'test': [
                'pytest',
                ],
            },
        entry_points={
            'console_scripts': [
                'talker = hw1_pubsub.publisher_member_function:main',
                'listener = hw1_pubsub.subscriber_member_function:main',
                ],
            },
        )
