cmake_minimum_required(VERSION 3.5)
project(fkie_mas_discovery)

# Update the policy setting to avoid an error when loading the ament_cmake package
# at the current cmake version level
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

find_package(ament_cmake QUIET)

if ( ament_cmake_FOUND )

    #---- Enable C++14 ----
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)

    if(MSVC)
        add_definitions(-D_CRT_SECURE_NO_WARNINGS)
    else()
        add_definitions(-Wpedantic)
    endif()

    add_definitions( -DUSING_ROS2 )
    message(STATUS "-------------------------------------------------\nFKIE MAS Discovery is being built using AMENT.\n-------------------------------------------------")

    # find dependencies
    find_package(fastrtps QUIET)
    find_package(fkie_mas_msgs REQUIRED)
    find_package(rclcpp REQUIRED)
    add_executable(mas-discovery
        src/eprosima_participant_listener.cpp
    )
    if (fastrtps_FOUND)
        # before ros kilted
        target_link_libraries(mas-discovery PUBLIC
            rclcpp::rclcpp
            fastrtps
            ${fkie_mas_msgs_TARGETS}
        )
    else()
        # since ros kilted
        find_package(fastdds REQUIRED)
        target_link_libraries(mas-discovery PUBLIC
            rclcpp::rclcpp
            fastdds
            ${fkie_mas_msgs_TARGETS}
        )
    endif()
    if(MSVC)
    else()
        target_compile_options(mas-discovery PRIVATE -Wall -Wextra -Werror=return-type)
    endif()
    
    set( MAS_DISCOVERY_LIB_DESTINATION   lib )
    set( MAS_DISCOVERY_INC_DESTINATION   include )
    set( MAS_DISCOVERY_BIN_DESTINATION   bin )
    set( MAS_DISCOVERY_SHARE_DESTINATION share/${PROJECT_NAME} )

    ament_export_libraries(${PROJECT_NAME})
    ament_package()
    install(TARGETS
        mas-discovery
        DESTINATION lib/${PROJECT_NAME})

elseif( CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE)

    find_package(catkin REQUIRED COMPONENTS
        fkie_mas_msgs
        fkie_mas_pylib
        rosgraph
        roslib
        rospy
        std_srvs
    )
    find_package(GTest)

    message(STATUS "--------------------------------------------------\nFKIE MAS Discovery is being built using CATKIN.\n--------------------------------------------------")

    catkin_python_setup()
    catkin_package( CATKIN_DEPENDS
        fkie_mas_msgs
        fkie_mas_pylib
        rosgraph
        roslib
        rospy
        std_srvs
    )
    set( MAS_DISCOVERY_LIB_DESTINATION   ${CATKIN_PACKAGE_LIB_DESTINATION} )
    set( MAS_DISCOVERY_INC_DESTINATION   ${CATKIN_GLOBAL_INCLUDE_DESTINATION} )
    set( MAS_DISCOVERY_BIN_DESTINATION   ${CATKIN_GLOBAL_BIN_DESTINATION} )
    set( MAS_DISCOVERY_SHARE_DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
    catkin_install_python(
        PROGRAMS
            nodes/mas-discovery
            nodes/mas-zeroconf
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

else()
    set( MAS_DISCOVERY_LIB_DESTINATION   lib )
    set( MAS_DISCOVERY_INC_DESTINATION   include )
    set( MAS_DISCOVERY_BIN_DESTINATION   bin )
    set( MAS_DISCOVERY_SHARE_DESTINATION share )

endif()

#############################################################

message( STATUS "MAS_DISCOVERY_LIB_DESTINATION:   ${MAS_DISCOVERY_LIB_DESTINATION} " )
message( STATUS "MAS_DISCOVERY_BIN_DESTINATION:   ${MAS_DISCOVERY_BIN_DESTINATION} " )
message( STATUS "MAS_DISCOVERY_SHARE_DESTINATION: ${MAS_DISCOVERY_SHARE_DESTINATION} " )

install(
    DIRECTORY
    launch
    DESTINATION ${MAS_DISCOVERY_SHARE_DESTINATION}
)
