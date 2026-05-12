from setuptools import find_packages, setup

package_name = 'main_state_repeater'

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
    maintainer='rsrmstayhard',
    maintainer_email='mingjiehu5@gmail.com',
    description='ROS2 node that manages the main state of the road surface repair machine',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'main_state_repeater = main_state_repeater.main_state_repeater:main',
        ],
    },
)
