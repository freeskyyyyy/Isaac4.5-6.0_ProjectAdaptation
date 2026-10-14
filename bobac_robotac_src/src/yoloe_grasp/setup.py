from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'yoloe_grasp'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'weights'), 
         glob('weights/*')),
         
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='shqd0w123@outlook.com',
    description='YOLOE segmentation + depth + TF grasp point publisher',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yoloe_grasp_node = yoloe_grasp.yoloe_grasp_node:main',
        ],
    },
)
