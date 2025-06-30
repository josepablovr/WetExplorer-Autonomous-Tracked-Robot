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
        ('share/' + package_name + '/config', [
            'config/yolo_config.yaml',
            'config/realsense_config.yaml',  # Added new RealSense config file
            'config/hdensity.json'                        
        ]),
        ('share/' + package_name + '/launch', [
            'launch/mask_launch.py',
            'launch/obstacle_detector_launch.py',
            'launch/vision_launch.py',            
            'launch/camera_launch.py'  # Added new RealSense launch file
        ]),
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
            'mask_node = wetexplorer_vision.mask_node:main',
            'image_processing = wetexplorer_vision.image_processing:main',                
        ],
    },
)