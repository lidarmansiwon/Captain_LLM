from setuptools import find_packages, setup
import os
import glob

package_name = 'captain_usv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='siwon',
    maintainer_email='kimsanmaro@naver.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'captain_node = captain_usv.captain_node:main',
        'rule_engine_node = captain_usv.rule_engine_node:main',
        'fsm_node = captain_usv.fsm_node:main',
        'coordinator_node = captain_usv.coordinator_node:main',
        'world_model_node = captain_usv.world_model_node:main',
        ],
    },
)
