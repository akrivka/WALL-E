from setuptools import find_packages, setup
from glob import glob

package_name = 'walle'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='walle',
    maintainer_email='walle@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ps5 = walle.ps5:main',
            'motors = walle.motors:main',
            'servos = walle.servos:main',
            'webserver = walle.webserver:main',
            'effects = walle.effects:main'
        ],
    },
)
