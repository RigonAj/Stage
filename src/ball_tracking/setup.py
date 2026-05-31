from setuptools import find_packages, setup

package_name = 'ball_tracking'

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
    maintainer='riajvazi',
    maintainer_email='rigonajvazi@proton.me',
    description='Python utilities for the ball tracking workspace.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_node = ball_tracking.camera_node:main',
        ],
    },
)
