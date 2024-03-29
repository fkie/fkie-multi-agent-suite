cmake_minimum_required(VERSION 3.5)
project(fkie_mas_sync)


# Update the policy setting to avoid an error when loading the ament_cmake package
# at the current cmake version level
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

find_package(ament_cmake QUIET)

if ( ament_cmake_FOUND )
    message(STATUS "------------------------------------------")
    message(STATUS "FKIE master sync is being ignore using AMENT.")
    message(STATUS "------------------------------------------")
    # only to avoid warnings while build
    message(STATUS "install prefix: ${CATKIN_INSTALL_INTO_PREFIX_ROOT}")
    message(STATUS "symlink install: ${CATKIN_SYMLINK_INSTALL}")

elseif( CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE)

    set(catkin_FOUND 1)
    add_definitions( -DUSING_ROS )

    message(STATUS "------------------------------------------")
    message(STATUS "FKIE master sync is being built using CATKIN.")
    message(STATUS "------------------------------------------")

    find_package(catkin REQUIRED COMPONENTS fkie_mas_discovery fkie_mas_msgs)

    catkin_package(CATKIN_DEPENDS fkie_mas_discovery fkie_mas_msgs)
    catkin_python_setup()

    catkin_install_python(
        PROGRAMS
            nodes/mas-sync
            nodes/mas-param-sync.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

    install(
        DIRECTORY
        launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

endif()
