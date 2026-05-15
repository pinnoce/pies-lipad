from setuptools import setup

package_name = 'pies_vision'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'green_x_detector = pies_vision.green_x_detector:main',
            'red_bullseye_detector = pies_vision.red_bullseye_detector:main',
        ],
    },
)
