from setuptools import setup

package_name = 'py_tutorial'

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
    maintainer='robotsky',
    maintainer_email='759094438@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "simple_node = py_tutorial.simple_node:main",
            "simple_client = py_tutorial.simple_client:main",
            "simple_server = py_tutorial.simple_server:main",
        ],
    },
)
