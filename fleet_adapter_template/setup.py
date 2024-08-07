from setuptools import setup

package_name = 'fleet_adapter_template'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name,['config.yaml']),

    ],
install_requires=['setuptools', 'fastapi>=0.79.0', 'uvicorn>=0.18.2'],
    zip_safe=True,
    maintainer='Xi Yu Oh',
    maintainer_email='xiyu@openrobotics.org',
    description='Fleet adapters for interfacing with RMF Demos robots with a '
                'fleet manager via REST API',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_adapter=fleet_adapter_template.fleet_adapter:main',
            'fleet_manager=fleet_adapter_template.fleet_manager:main',
        ],
    },
)
