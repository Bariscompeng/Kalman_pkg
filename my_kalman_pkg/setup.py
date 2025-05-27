from setuptools import setup

package_name = 'my_kalman_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name + '/data', ['data/measurements.csv']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Barış',
    maintainer_email='baris@example.com',
    description='Kalman filter uygulaması',
    license='MIT',
    entry_points={
        'console_scripts': [
            'csv_publisher = my_kalman_pkg.csv_publisher:main',
            'kalman_node = my_kalman_pkg.kalman_node:main',
        ],
    },
)

