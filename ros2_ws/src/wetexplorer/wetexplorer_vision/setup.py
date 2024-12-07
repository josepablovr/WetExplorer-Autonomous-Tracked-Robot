from setuptools import find_packages, setup

package_name = 'wetexplorer_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/config', ['config/obstacle_params.yaml']),
    ('share/' + package_name + '/launch', ['launch/obstacle_detector_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'vision_node = wetexplorer_vision.vision_node:main',
        'obstacle_finder = wetexplorer_vision.obstacle_detector:main',
    ],
},

)