cmake_minimum_required(VERSION 3.5)
project(fkie_mas_discovery)

#---- Add the subdirectory cmake ----
#set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
#list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

#---- Enable C++14 ----
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
else()
    add_definitions(-Wpedantic)
endif()

#set(CMAKE_POSITION_INDEPENDENT_CODE ON)


# Update the policy setting to avoid an error when loading the ament_cmake package
# at the current cmake version level
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

find_package(ament_cmake QUIET)

if ( ament_cmake_FOUND )

    add_definitions( -DUSING_ROS2 )
    message(STATUS "-------------------------------------------------\nFKIE MAS Discovery is being built using AMENT.\n-------------------------------------------------")

    set(BUILD_TOOL_INCLUDE_DIRS ${ament_INCLUDE_DIRS})

    # find dependencies
    find_package(rclcpp REQUIRED)
    find_package(rclcpp_lifecycle REQUIRED)
    find_package(rmw_fastrtps_cpp REQUIRED)
    find_package(rmw_fastrtps_shared_cpp REQUIRED)
    find_package(lifecycle_msgs REQUIRED)
    find_package(fkie_mas_msgs REQUIRED)

elseif( CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE)

    find_package(catkin REQUIRED COMPONENTS fkie_mas_msgs fkie_mas_pylib rosgraph roslib rospy std_srvs)
    find_package(GTest)

    message(STATUS "--------------------------------------------------\nFKIE MAS Discovery is being built using CATKIN.\n--------------------------------------------------")

    catkin_python_setup()
    catkin_package( CATKIN_DEPENDS fkie_mas_msgs fkie_mas_pylib rosgraph roslib rospy std_srvs )

endif()

if ( ament_cmake_FOUND )
    set(DEPS
        fkie_mas_msgs
        rclcpp
        rclcpp_lifecycle
        rmw_fastrtps_shared_cpp
        lifecycle_msgs
    )
    list(APPEND DEPS rmw_fastrtps_cpp)
    include_directories(${catkin_INCLUDE_DIRS})
    add_executable(mas-discovery
        src/eprosima_pariticipant_listener.cpp
    )
    ament_target_dependencies(mas-discovery ${DEPS})


elseif( CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE)

endif()

#############################################################
if(ament_cmake_FOUND)
    if(MSVC)
    else()
        target_compile_options(mas-discovery PRIVATE
            -Wall -Wextra -Werror=return-type)
    endif()
    
    #find_package(ament_index_cpp REQUIRED)
    #ament_target_dependencies(${PROJECT_NAME} PUBLIC ament_index_cpp)
    #ament_export_dependencies(ament_index_cpp)

    set( MAS_DISCOVERY_LIB_DESTINATION   lib )
    set( MAS_DISCOVERY_INC_DESTINATION   include )
    set( MAS_DISCOVERY_BIN_DESTINATION   bin )
    set( MAS_DISCOVERY_SHARE_DESTINATION share/${PROJECT_NAME} )

    ament_export_libraries(${PROJECT_NAME})
    ament_package()
    install(TARGETS
        mas-discovery
        DESTINATION lib/${PROJECT_NAME})
elseif(catkin_FOUND)
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

message( STATUS "MAS_DISCOVERY_LIB_DESTINATION:   ${MAS_DISCOVERY_LIB_DESTINATION} " )
message( STATUS "MAS_DISCOVERY_BIN_DESTINATION:   ${MAS_DISCOVERY_BIN_DESTINATION} " )
message( STATUS "MAS_DISCOVERY_SHARE_DESTINATION: ${MAS_DISCOVERY_SHARE_DESTINATION} " )

install(
    DIRECTORY
    launch
    DESTINATION ${MAS_DISCOVERY_SHARE_DESTINATION}
)
