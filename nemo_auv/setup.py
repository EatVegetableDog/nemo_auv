from setuptools import find_packages, setup

package_name = 'nemo_auv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml',
                                    'config/auv_cam_params.yaml',]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ishanin',
    maintainer_email='ishani.narwankar@gmail.com',
    description='check the camera feed and control the nemo auv',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['camera_feed = nemo_auv.camera_feed:main',],
    },
)
