from setuptools import find_packages, setup

package_name = 'occupancy_grid_processor'

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
    maintainer='sally',
    maintainer_email='pascal.sikorski@slu.edu',
    description='occupancy_grid_processor',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'occupancy_grid_processor = occupancy_grid_processor.occupancy_grid_processor:main'
        ],
    },
)
