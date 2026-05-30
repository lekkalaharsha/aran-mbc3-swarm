from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'radar_fusion'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),  glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),  glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'numpy', 'scikit-learn', 'joblib'],
    zip_safe=True,
    maintainer='Boson Motors',
    maintainer_email='eeindia@bosonmotors.com',
    description='MBC-3 radar detection and swarm fusion',
    license='MIT',
    entry_points={
        'console_scripts': [
            'detection_node = radar_fusion.detection_node:main',
            'fusion_node    = radar_fusion.fusion_node:main',
        ],
    },
)
