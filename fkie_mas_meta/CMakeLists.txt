cmake_minimum_required(VERSION 3.5)
project(fkie_mas_meta)


# Update the policy setting to avoid an error when loading the ament_cmake package
# at the current cmake version level
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

find_package(ament_cmake QUIET)

if ( ament_cmake_FOUND )

    add_definitions( -DUSING_ROS2 )
    message(STATUS "--------------------------------------------------\nFKIE multi-agent-suite is being built using AMENT.\n--------------------------------------------------")
    # only to avoid warnings while build
    message(STATUS "install prefix: ${CATKIN_INSTALL_INTO_PREFIX_ROOT}")
    message(STATUS "symlink install: ${CATKIN_SYMLINK_INSTALL}")
    # find dependencies
    ament_package()

elseif( CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE)

    find_package(catkin REQUIRED )
    message(STATUS "---------------------------------------------------\nFKIE multi-agent-suite is being built using CATKIN.\nexport compile commads: ${CMAKE_EXPORT_COMPILE_COMMANDS}\n---------------------------------------------------")

    catkin_package()

endif()
