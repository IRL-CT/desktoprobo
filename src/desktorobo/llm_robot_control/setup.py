import os
from setuptools import setup
from glob import glob

package_name = 'llm_robot_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'web'), glob('llm_robot_control/web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='qingxuan',
    maintainer_email='qy264@cornell.edu',
    description='Natural-language robot control via OpenAI tool use + web UI.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_controller_node = llm_robot_control.llm_controller_node:main'
        ],
    },
)
