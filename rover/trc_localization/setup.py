from glob import glob

from setuptools import find_packages, setup

package_name = 'trc_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ryo Shimano',
    maintainer_email='richiesss@users.noreply.github.com',
    description='GNSS と LiDAR オドメトリを融合した位置推定',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'heading_init_node = trc_localization.heading_init_node:main',
            'fake_sensors_node = trc_localization.fake_sensors_node:main',
        ],
    },
)
