from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'robot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')), # pour lancer le launch file meme hors du dossier launch, il aut indiquer ca presence ici
   
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='turtle',
    maintainer_email='sayfchafikpro@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "navigation = robot_navigation.robot_navigationR:main",
            #"navigation2 = robot_navigation.navigation8:main",
        ],
    },
)
