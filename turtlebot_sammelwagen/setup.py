from setuptools import find_packages, setup

package_name = 'yolo_object_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/approach_with_yolo.launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='boris',
    maintainer_email='boris@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'approach_with_yolo = yolo_object_detection.approach_with_yolo:main',
            'ultraschall_node = yolo_object_detection.ultraschall_node:main',
            'test_drive_node = yolo_object_detection.test_drive_node:main'

        ],
    },
)

