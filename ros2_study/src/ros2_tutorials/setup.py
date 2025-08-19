from setuptools import find_packages, setup

package_name = 'encoder_pkg'

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
    maintainer='min',
    maintainer_email='coghd6400@gmail.com',
    description='Encoder publisher using pigpio or mock',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_pub = encoder_pkg.encoder_publisher:main',
        ],
    },
)

