cmake_minimum_required(VERSION 3.5)
project(fkie_mas_msgs)

# Update the policy setting to avoid an error when loading the ament_cmake package
# at the current cmake version level
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

find_package(ament_cmake QUIET)

if ( ament_cmake_FOUND )

    add_definitions( -DUSING_ROS2 )
    message(STATUS "-----------------------------------------\nFKIE mas_msgs is being built using AMENT.\n-----------------------------------------")

    find_package(builtin_interfaces REQUIRED)
    find_package(rosidl_default_generators REQUIRED)

    rosidl_generate_interfaces(${PROJECT_NAME}
      "msg2/DiscoveredState.msg"
      "msg2/Endpoint.msg"
      "msg2/Gid.msg"
      "msg2/ParticipantEntitiesInfo.msg"
      "msg2/Qos.msg"
      "srv/ListNodes.srv"
      "srv/LoadLaunch.srv"
      "srv/Task.srv"
      DEPENDENCIES builtin_interfaces
    )

    ament_package()
elseif( CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE)

    message(STATUS "------------------------------------------\nFKIE mas_msgs is being built using CATKIN.\n------------------------------------------")

    find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

    #######################################
    ## Declare ROS messages and services ##
    #######################################

    ## Generate messages in the 'msg' folder
    add_message_files(
      DIRECTORY msg
      FILES
      LinkState.msg
      LinkStatesStamped.msg
      MasterState.msg
      ROSMaster.msg
      SyncMasterInfo.msg
      SyncServiceInfo.msg
      SyncTopicInfo.msg
    )

    ## Generate services in the 'srv' folder
    add_service_files(
      DIRECTORY srv
      FILES
      DiscoverMasters.srv
      GetSyncInfo.srv
      ListNodes.srv
      LoadLaunch.srv
      Task.srv
    )

    #catkin_python_setup()
    generate_messages(DEPENDENCIES std_msgs)

    catkin_package(
        CATKIN_DEPENDS message_runtime std_msgs
    )

endif()
