import os
from setuptools import setup
from glob import glob

package_name = 'mobile_robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    # package_dir={"":"src"},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='frankbu',
    maintainer_email='fb266@cornell.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mobile_robot_control_node = mobile_robot_control.mobile_robot_control_node:main'
        ],
    },
)
