from setuptools import find_packages, setup

package_name = 'euler_to_quat'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name +'/launch',['launch/euler_to_quat.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ashish Sukumar',
    maintainer_email='asukumar1@wpi.edu',
    description='A node to convert euler angles to quaternion representations',
    license='Apache2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'euler_to_quat = euler_to_quat.angle_sub:main' 
        
        ],
    },
)
