from setuptools import setup

package_name = 'example_action_py'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "fibonacci_action_server = example_action_py.fibonacci_action_server:main",
            "fibonacci_action_client = example_action_py.fibonacci_action_client:main",
        ],
    },
)
