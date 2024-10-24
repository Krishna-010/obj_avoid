from setuptools import find_packages, setup

package_name = 'obj_avoid'

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
    maintainer='bossoflords',
    maintainer_email='nithin2002krishna@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'gtg=obj_avoid.go_to_goal:main',
        	'obj_av=obj_avoid.object_avoidance:main',
        	'range_det=obj_avoid.range_detection:main',
        	'tf=obj_avoid.transformation:main',
        	'way_load=obj_avoid.waypoint_loader:main',
        ],
    },
)
