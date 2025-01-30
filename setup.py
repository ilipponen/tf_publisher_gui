from setuptools import find_packages, setup

package_name = 'tf_publisher_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', ['launch/demo.launch.py']),
        (f'share/{package_name}/rviz', ['rviz/demo.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ilkka',
    maintainer_email='15342874+ilipponen@users.noreply.github.com',
    description='Simple GUI for publishing tf transforms',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_publisher_gui = tf_publisher_gui.tf_publisher_gui:main',
        ],
    },
)
