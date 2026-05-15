from setuptools import setup

package_name = 'pies_servo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'servo_node = pies_servo.servo_node:main',
        ],
    },
)
