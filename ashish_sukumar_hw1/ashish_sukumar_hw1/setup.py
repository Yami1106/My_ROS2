from setuptools import find_packages, setup

package_name = 'ashish_sukumar_hw1'

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
    maintainer='Ashish Sukumar',
    maintainer_email='asukumar1@wpi.edu',
    description='A publisher which publishes increasing integer values every second and a subscriber checks if the integer is odd or even and prints it in the terminal',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = ashish_sukumar_hw1.integer_publisher:main',
            'listener = ashish_sukumar_hw1.integer_subscriber:main'
        ],
    },
)
