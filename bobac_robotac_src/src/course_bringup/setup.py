from glob import glob
from setuptools import find_packages, setup

setup(
    name='course_bringup', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/course_bringup']),
        ('share/course_bringup', ['package.xml']),
        ('share/course_bringup/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Course Maintainer', maintainer_email='course@example.invalid',
    description='Isaac Sim 6.0 course bringup', license='Apache-2.0',
    entry_points={'console_scripts': ['course_readiness = course_bringup.readiness:main']},
)
