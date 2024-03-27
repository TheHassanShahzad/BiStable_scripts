from setuptools import find_packages, setup

package_name = 'BiStable_scripts'

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
    maintainer='the_hassan_shahzad',
    maintainer_email='hshahzad2005108277@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "gamepad_publishe_twist = BiStable_scripts.gamepad_publishe_twist:main",
            "gamepad_publisher = BiStable_scripts.gamepad_publisher:main",
            "tracking_publisher = BiStable_scripts.hand_tracking:main",
            "combiner_node = BiStable_scripts.combiner_node:main"
        ],
    },
)
