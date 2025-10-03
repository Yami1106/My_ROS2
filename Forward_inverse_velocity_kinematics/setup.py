from setuptools import find_packages, setup

package_name = 'Forward_kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','numpy'],
    zip_safe=True,
    maintainer='Ashishh Sukumar',
    maintainer_email='asukumar1@wpi.edu',
    description='Calculate the end-effector pose for a given robot configuration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'Fwd_km_node = Forward_kinematics.fwd_km:main'
        ],
    },
)
