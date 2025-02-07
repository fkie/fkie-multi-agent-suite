import os

package_name = 'fkie_mas_daemon'

if 'ROS_VERSION' in os.environ and os.environ['ROS_VERSION'] == '1':
    from setuptools import setup
    from catkin_pkg.python_setup import generate_distutils_setup

    d = generate_distutils_setup(
        # don't do this unless you want a globally visible script
        # scripts=['nodes/mas-daemon', 'nodes/mas-subscriber'],
        packages=[package_name, f'{package_name}.monitor'],
        package_dir={'': 'src'}
    )

    setup(**d)
    exit(0)

else:
    ### ROS2 ###
    import xml.etree.ElementTree as ET
    from setuptools import setup

    resource_files = [
        'tests/resources/description_example.launch',
        'tests/resources/include_dummy.launch',
        'tests/resources/included1.launch',
        'tests/resources/included2.launch',
    ]

    version = "0.0.0"

    def get_version():
        tree = ET.parse('package.xml')
        root = tree.getroot()
        for vers in root.findall('version'):
            return vers.text
        return version

    def strip_dirty_vers(vers):
        parts = vers.lstrip('v').split('-', 2)
        return parts[0]

    setup(
        name=package_name,
        version=strip_dirty_vers(get_version()[0]),
        packages=[package_name, f'{package_name}.monitor'],
        data_files=[
            ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
            (f'share/{package_name}', ['package.xml']),
            (f'share/{package_name}/test/launch',
             ['tests/launch/test_composable_launch.py', 'tests/launch/test_included_launch.py']),
            # (f'share/{package_name}/tests/resources', resource_files),
            (f'lib/{package_name}',
             ['scripts/mas-remote-node.py', 'scripts/mas-respawn'])
        ],
        install_requires=['setuptools', 'ruamel.yaml', 'launch-xml'],
        zip_safe=True,
        maintainer='Alexander Tiderko',
        maintainer_email='Alexander.Tiderko@fkie.fraunhofer.de',
        description='A daemon node to manage ROS launch files and launch nodes from loaded files.',
        license='MIT',
        url='https://github.com/fkie/ros_node_manager',
        tests_require=['pytest'],
        test_suite="tests",
        entry_points={
            'console_scripts': [
                'mas-daemon ='
                ' fkie_mas_daemon:main',
                'mas-subscriber ='
                ' fkie_mas_daemon:subscriber',
            ],
        },
    )
