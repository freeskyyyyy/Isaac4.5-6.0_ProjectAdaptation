from setuptools import find_packages, setup

package_name = 'grasp_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Grasp control state machine and gripper control',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grasp_state_machine = grasp_control.grasp_state_machine:main',
            'place_state_machine = grasp_control.place_state_machine:main',
            'gripper_control_node = grasp_control.gripper_control_node:main',
        ],
    },
)
