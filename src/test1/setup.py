from setuptools import find_packages, setup

package_name = 'test1'

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
    maintainer='control',
    maintainer_email='gidijalaraja@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sub_phantom_button = test1.sub_phantom_button:main',
            'sub_phantom_force = test1.sub_phantom_force:main',
            'sub_phantom_joint_state = test1.sub_phantom_joint_state:main',
            'sub_phantom_pose = test1.sub_phantom_pose:main',
            'sub_phantom_state = test1.sub_phantom_state:main',
        ],
    },
)
