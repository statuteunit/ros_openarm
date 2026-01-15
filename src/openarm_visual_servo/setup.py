from setuptools import find_packages, setup

package_name = 'openarm_visual_servo'

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
    maintainer='statute',
    maintainer_email='statuteunit@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'get_poses = openarm_visual_servo.get_poses:main',
        'camera_vision = openarm_visual_servo.camera_vision:main',
        'openarm_grasp_banana = openarm_visual_servo.openarm_grasp_banana:main',
        'openarm_joint_state_subscriber = openarm_visual_servo.openarm_joint_state_subscriber:main',
        'publish_test_pose = openarm_visual_servo.publish_test_pose:main',
        ],
    },
)
