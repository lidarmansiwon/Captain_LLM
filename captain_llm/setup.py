from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'captain_llm'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'webapp'), glob('webapp/*'))
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
            # 'captain_llm = captain_llm.rosgpt:main',
            # 'captain_llmparser_turtlesim = captain_llm.captain_llmparser_turtlesim:main',
            # 'captain_llm_client_node = captain_llm.captain_llm_client_node:main',
            'captain_node = captain_llm.captain_node:main',
            'subcaptain_node = captain_llm.subcaptain_node:main',
            'server_node = captain_llm.server_node:main',
            'nav_guardian = captain_llm.nav_guardian:main',
            'rosgpt = captain_llm.rosgpt:main',
            'rosgpt_client_node = captain_llm.rosgpt_client_node:main',
            'usv_control.py = captain_llm.usv_control:main',
            
        ],
    },
)
