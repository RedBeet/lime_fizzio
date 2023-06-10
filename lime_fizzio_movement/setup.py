from setuptools import setup

package_name = 'lime_fizzio_movement'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mingun',
    maintainer_email='ohmingun9@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lime_fizzio_motor = lime_fizzio_movement.lime_fizzio_motor:main'
            'lime_fizzio_sensor = lime_fizzio_movement.lime_fizzio_sensor:main'
        ],
    },
)
