import os
from glob import glob
from setuptools import setup

package_name = 'suave_metacontrol'

setup(
    name=package_name,
    version='0.23.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeroen',
    maintainer_email='j.zwanepol@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_managed_system = ' +
                'suave_metacontrol.fake_managed_system:main',
            'suave_reasoner = suave_metacontrol.suave_reasoner:main',
            'metacontrol_goal_update = ' +
                'suave_metacontrol.metacontrol_goal_update:main',
            'random_reasoner = '+
                'suave_metacontrol.random_reasoner:main',
        ],
    },
)
