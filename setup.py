from setuptools import setup
import os
from glob import glob

package_name = 'servo_hoge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ashikaga',
    maintainer_email='matsuta2718@gmail.com',
    description='servo test',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = servo_hoge.controller:main',
            'servo_2 = servo_hoge.servo_2:main',
            #'servo = servo_hoge.servo:main',
            #'led = servo_hoge.led:main',
        ],
    },
)
