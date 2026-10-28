from setuptools import find_packages, setup

package_name = 'lab1_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/lab1.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='william-amir',
    maintainer_email='william-amir@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'turtle_controller = lab1_control.turtle_controller:main',
        'square_server = lab1_control.square_server:main',
    ],
},
)

