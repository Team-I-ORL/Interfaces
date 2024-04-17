from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'openai_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'audio'), glob(os.path.join('audio', '*.*'))),    
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jinkai',
    maintainer_email='jinkaiq@andrew.cmu.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'openaiTTS = openai_ros2.openaiTTS:main',
        ],
    },
)
