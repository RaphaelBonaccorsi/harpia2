from setuptools import setup, find_packages

package_name = 'mission_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raphael',
    maintainer_email='raphael@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_planner_server = mission_planning.mission_planner_server:main',
            'test_add_goal = mission_planning.test_add_goal:main',
            'test_client_copy = mission_planning.test_client_copy:main',
            'test_client = mission_planning.test_client:main',
        ],
    },
)

