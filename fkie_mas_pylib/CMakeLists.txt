cmake_minimum_required(VERSION 3.5)
project(fkie_mas_pylib)

# Update the policy setting to avoid an error when loading the ament_cmake package
# at the current cmake version level
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

find_package(ament_cmake QUIET)

if ( ament_cmake_FOUND )
elseif( CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE)

    message(STATUS "-------------------------------------------------\nFKIE MAS pylib is being built using CATKIN.\n-------------------------------------------------")

    find_package(catkin REQUIRED COMPONENTS
      fkie_mas_msgs
      rospy
    )

    catkin_python_setup()

    catkin_package(
      CATKIN_DEPENDS
      fkie_mas_msgs
      rospy
    )

    # # Add folders to be run by python nosetests
    if(CATKIN_ENABLE_TESTING)
        add_subdirectory(${PROJECT_NAME}/tests)
    endif()

endif()
