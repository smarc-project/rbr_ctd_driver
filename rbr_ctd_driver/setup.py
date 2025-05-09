from setuptools import find_packages, setup
import glob
import os

package_name = 'rbr_ctd_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob.glob('launch/*.launch')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Li Ling',
    maintainer_email='liling@kth.se',
    description='CTD driver package for RBR CTD',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rbr_ctd_driver = rbr_ctd_driver.parse_rbr_ctd:main',
            'serial_reader_ctd = rbr_ctd_driver.serial_reader_ctd:main',
        ],
    },
)
