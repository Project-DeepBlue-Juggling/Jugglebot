from setuptools import find_packages, setup
import glob

package_name = 'jugglebot_simulator'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/launch.py']))
data_files.append(('share/' + package_name + '/worlds', list(glob.glob('worlds/*'))))
data_files.append(('share/' + package_name + '/resource', ['resource/jugglebot.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='josh',
    maintainer_email='jridd30@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jugglebot_driver = jugglebot_simulator.jugglebot_driver:main'
        ],
    },
)
