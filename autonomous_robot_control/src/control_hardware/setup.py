from setuptools import find_packages, setup

package_name = 'control_hardware'

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
    maintainer='tamir',
    maintainer_email='tamirbasson99@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mux = control_hardware.mux:main',
            'cmd2pwm = control_hardware.cmd2pwm:main',
            'pwm2GPIO = control_hardware.pwm2GPIO:main'
            
        ],
    },
)
