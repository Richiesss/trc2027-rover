from setuptools import find_packages, setup

package_name = 'trc_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ryo Shimano',
    maintainer_email='richiesss@users.noreply.github.com',
    description='LiDAR の点群から障害物を見つける',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'obstacle_detector_node = trc_perception.obstacle_detector_node:main',
        ],
    },
)
