from setuptools import find_packages, setup

package_name = 'wetexplorer_vision_predator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/convert_launch.py']),
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
        'model_converter = wetexplorer_vision_predator.model_converter:main',
        'pose = wetexplorer_vision_predator.publish_pose_node:main'
    ],
},

)
