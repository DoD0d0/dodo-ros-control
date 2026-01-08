from setuptools import setup, find_packages

package_name = 'dodo_ros_control'

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
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS 2 runtime for sim2sim and sim2real control of the Dodo robot.',
    license='TODO',
    entry_points={
    "console_scripts": [
        "inference_node = dodo_ros_control.inference_node:main",
        "dummy_node = dodo_ros_control.dummy_node:main",
    ],
},
)