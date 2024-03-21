cmake_minimum_required(VERSION 3.5)
project(fkie_mas_daemon)

# Update the policy setting to avoid an error when loading the ament_cmake package
# at the current cmake version level
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

find_package(ament_cmake QUIET)

if(ament_cmake_FOUND)
elseif(CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE)
    find_package(catkin REQUIRED COMPONENTS
        diagnostic_msgs
        fkie_mas_msgs
        fkie_mas_pylib
        roslaunch
        rospy
    )

    catkin_python_setup()
    catkin_package(
        CATKIN_DEPENDS
        diagnostic_msgs
        fkie_mas_msgs
        fkie_mas_pylib
        roslaunch
        rospy
    )

    # ############
    # # Install ##
    # ############
    catkin_install_python(
        PROGRAMS
        nodes/mas-daemon
        nodes/mas-subscriber
        scripts/mas-remote-node.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

    install(
        PROGRAMS
        scripts/mas-respawn
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

    install(
        DIRECTORY
        launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

    install(
        DIRECTORY
        tests/resources
        tests/resources_alt
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
    )

    # # Add folders to be run by python nosetests
    if(CATKIN_ENABLE_TESTING)
        add_subdirectory(tests)
    endif()
endif()
