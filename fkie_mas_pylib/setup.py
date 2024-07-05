import os

package_name = 'fkie_mas_pylib'

if 'ROS_VERSION' in os.environ and os.environ['ROS_VERSION'] == '1':
    from setuptools import setup
    from catkin_pkg.python_setup import generate_distutils_setup

    d = generate_distutils_setup(
        # don't do this unless you want a globally visible script
        scripts=[],
        packages=[package_name,
                  package_name + '.interface',
                  package_name + '.launch',
                  package_name + '.logging',
                  package_name + '.parameters',
                  package_name + '.system',
                  package_name + '.websocket'],
        package_dir={'': '.'}
    )

    setup(**d)
    exit(0)
else:
    ### ROS2 ###
    from setuptools import setup
    import xml.etree.ElementTree as ET

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
        packages=[package_name,
                  package_name + '.interface',
                  package_name + '.launch',
                  package_name + '.logging',
                  package_name + '.parameters',
                  package_name + '.system',
                  package_name + '.websocket'],
        data_files=[
            ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml'])
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='Alexander Tiderko',
        maintainer_email='Alexander.Tiderko@fkie.fraunhofer.de',
        description='Python helpers required by multi agent suite packages.',
        license='MIT',
        url='https://github.com/fkie/multimaster_fkie',
        tests_require=['pytest'],
        test_suite="tests",
    )
