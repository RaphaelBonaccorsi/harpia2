from setuptools import setup, find_packages

package_name = 'mission_goal_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Raphael M. C. Bonaccorsi',
    maintainer_email='raphaelbonaccorsi@usp.br',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_goal_manager_server_temporal_planing = mission_goal_manager.mission_goal_manager_server_temporal_planing:main',
            'mission_goal_manager = mission_goal_manager.mission_goal_manager_server:main'
        ],
    },
)

