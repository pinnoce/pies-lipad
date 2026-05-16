from setuptools import setup

package_name = 'pies_mission'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/mission.launch.py',
            'launch/autonomous.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'visual_centering = pies_mission.visual_centering:main',
            'autonomous_mission = pies_mission.autonomous_mission:main',
            'calibrate_gains = pies_mission.calibrate_gains:main',
        ],
    },
)
