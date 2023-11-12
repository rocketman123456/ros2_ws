from setuptools import setup

package_name = 'example_srvcli_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rocketsky',
    maintainer_email='759094438@qq.com',
    description='Python client server tutorial',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = example_srvcli_py.service_member_function:main',
            'client = example_srvcli_py.client_member_function:main',
        ],
    },
)
