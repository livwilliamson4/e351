from setuptools import find_packages, setup

package_name = 'e351'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_beacon = e351.detect_beacon:main',
            'detectv2 = e351.detectv2:main',
            'tests = e351.tests:main',
            'mode_changer = e351.mode_changer:main',
            'testingpid = e351.testingpid:main',
        ],
    },
)
