from setuptools import setup, find_packages

package_name = 'mission_fault_mitigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',               # ROS 2 Python client library
        'plansys2',            # Plansys2 library
        'plansys2_msgs',       # Plansys2 messages
    ],
    zip_safe=True,
    maintainer='Raphael M. C. Bonaccorsi',
    maintainer_email='raphaelbonaccorsi@usp.br',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_fault_mitigation_server = mission_fault_mitigation.mission_fault_mitigation_server:main'
        ],
    },
)