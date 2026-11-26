from setuptools import find_packages, setup

package_name = 'trc_logging'

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
    description='大会提出用の制御履歴を CSV に記録する',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'control_history_node = trc_logging.control_history_node:main',
        ],
    },
)
