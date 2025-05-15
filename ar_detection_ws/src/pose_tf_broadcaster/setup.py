from setuptools import setup

package_name = 'pose_tf_broadcaster'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[],
    data_files=[
        ('share/' + package_name + '/launch', ['launch/visualization_launch.py']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TF broadcasters for camera and marker from OptiTrack and ArUco.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_tf_broadcaster = scripts.pose_tf_broadcaster:main',
            'aruco_tf_broadcaster = scripts.aruco_tf_broadcaster:main',
        ],
    },
)

