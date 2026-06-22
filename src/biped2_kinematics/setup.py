from setuptools import setup

package_name = 'biped2_kinematics'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Pinocchio FK/IK nodes for biped2.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'foot_fk_publisher = biped2_kinematics.foot_fk_publisher:main',
            'foot_lift_demo = biped2_kinematics.foot_lift_demo:main',
        ],
    },
)
