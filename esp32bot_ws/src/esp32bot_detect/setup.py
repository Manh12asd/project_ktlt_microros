from setuptools import find_packages, setup
from glob import glob

package_name = 'esp32bot_detect'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='manh',
    maintainer_email='manh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'droidcam_publisher = esp32bot_detect.droidcam_publisher:main',
            'pingpong_detect = esp32bot_detect.pingpong_detect:main',
            'human_detect = esp32bot_detect.human_detect:main',
        ],
    },
)
