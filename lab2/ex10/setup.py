from setuptools import find_packages, setup

package_name = 'text_to_cmd_vel'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name] if False else []),  # можно оставить пустым, если нет ресурса
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stasesonchik',
    maintainer_email='s.malyarchuk@g.nsu.ru',
    description='Package with node translating text to cmd_vel',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'text_to_cmd_vel_node = text_to_cmd_vel.text_to_cmd_vel_node:main'
        ],
    },
)
