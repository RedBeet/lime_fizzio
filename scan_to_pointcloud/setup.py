from setuptools import setup

package_name = 'scan_to_pointcloud'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',
                      'laser_geometry'],
    zip_safe=True,
    maintainer='mingun',
    maintainer_email='mingun@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "scan_to_pointcloud = scan_to_pointcloud.scan_to_pointcloud:main"
        ],
    },
)
