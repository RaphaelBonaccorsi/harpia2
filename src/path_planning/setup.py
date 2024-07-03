from setuptools import setup, find_packages

package_name = 'path_planning'

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
            'path_planning_server = path_planning.path_planning_server:main',
        ],
    },
)

