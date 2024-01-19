cmake_minimum_required(VERSION 3.5)
project(fkie_mas_gui)


# Update the policy setting to avoid an error when loading the ament_cmake package
# at the current cmake version level
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

find_package(ament_cmake QUIET)

if ( ament_cmake_FOUND )

    add_definitions( -DUSING_ROS2 )
    message(WARNING " -----------------------------------------------------\n use yarn to build FKIE MAS gui. \n -----------------------------------------------------")
    # only to avoid warnings while build
    ament_package()

elseif( CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE)

    # set(catkin_FOUND 1)
    # add_definitions( -DUSING_ROS )

    message(WARNING " -----------------------------------------------------\n use yarn to build FKIE MAS gui. \n -----------------------------------------------------")
    # only to avoid warnings while build
    find_package(catkin REQUIRED)
    catkin_package()

endif()
