from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'web_control'

setup(
    name=package_name,  # must match folder/package name exactly
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    include_package_data=True,  # include package_data
    package_data={
        # Include HTML templates and any static files if needed
        'web_control': [
            'templates/*.html',
            'static/*.*',  # optional if you have CSS/JS
        ],
    },
    data_files=[
        # Required for ROS2
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'flask',  # Flask is needed
    ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Web control interface for robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_gui=web_control.web_gui:main',
            'cmd_val_for_ard=web_control.cmd_val_for_ard:main',
        ],
    },
)
