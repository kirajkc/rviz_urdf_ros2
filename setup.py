from setuptools import find_packages, setup

package_name = 'ml3_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ml3',
    maintainer_email='ml3@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "qr_data = ml3_ros2.qr_data:main",
            "imu_data_v1 = ml3_ros2.imu_data_v1:main",
            "ultra_data = ml3_ros2.ultra_data:main",
            "ir_read_data = ml3_ros2.ir_read_data:main",
            "ir_data_sub = ml3_ros2.ir_data_sub:main",
            "qr_sub = ml3_ros2.qr_sub:main",
            "imu_sub = ml3_ros2.imu_sub:main",
            "ultra_sub = ml3_ros2.ultra_sub:main",
            "imu = ml3_ros2.imu:main",
            "keyboard = ml3_ros2.keyboard:main",
            "rviz = ml3_ros2.rviz:main",
            "traffic_light = ml3_ros2.traffic_light:main",
            "traffic_sub = ml3_ros2.traffic_sub:main",
        ],
    },
)
