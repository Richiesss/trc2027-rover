from setuptools import find_packages, setup

package_name = 'trc_navigation'

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
    description='障害物をよけながら目標地点へ向かう',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'local_planner_node = trc_navigation.local_planner_node:main',
            'mission_node = trc_navigation.mission_node:main',
        ],
    },
)
