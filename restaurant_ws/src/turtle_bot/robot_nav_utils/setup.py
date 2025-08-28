from setuptools import find_packages, setup
import glob

package_name = 'robot_nav_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob.glob('config/pose.yaml')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='balaji',
    maintainer_email='balaji@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move = robot_nav_utils.move_to:main',
        ],
    },
)
