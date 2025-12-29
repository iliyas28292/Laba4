from setuptools import find_packages, setup
from glob import glob

package_name = 'lesson_04'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alexGT555',
    maintainer_email='alexGT555@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'step_0 = lesson_04.step_0:main',
            'step_1 = lesson_04.step_1:main',
            'step_2 = lesson_04.step_2:main',
            'step_3 = lesson_04.step_3:main',
            'square = lesson_04.square:main',
            'mover = lesson_04.mover:main',
            'draw_ilyas = lesson_04.draw_ilyas:main', # <--- добавляем свой файл выполнения кода 
        ],
    },
)
