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
            'servo = servo_hoge.servo:main'
        ],
    },
)
