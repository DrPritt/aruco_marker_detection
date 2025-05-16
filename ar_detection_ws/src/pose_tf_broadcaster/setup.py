from setuptools import setup, find_packages

package_name = 'pose_tf_broadcaster'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, package_name + '.*']),
    install_requires=['setuptools', 'rclpy', 'tf2_ros'],
    data_files=[
        ('share/' + package_name + '/launch', ['launch/visualization_launch.py']),
        ('share/' + package_name, ['package.xml']),
    ],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TF broadcasters for OptiTrack poses.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'optitrack_tf_broadcaster = pose_tf_broadcaster.optitrack_tf_broadcaster:main',
            'aruco_tf_broadcaster = pose_tf_broadcaster.aruco_tf_broadcaster:main',
        ],
    },
)

