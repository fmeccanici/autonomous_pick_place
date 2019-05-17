cmake_minimum_required(VERSION 2.8.3)
project(autonomous_pick_place)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs geometry_msgs message_filters message_generation dynamic_reconfigure rosbag dynamixel_workbench_msgs eigen_conversions moveit_ros_planning_interface moveit_visual_tools)


add_message_files(
  FILES
  benchmarking.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  moveit_msgs
)


catkin_package(CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs message_filters message_generation dynamic_reconfigure rosbag dynamixel_workbench_msgs)


include_directories(${catkin_INCLUDE_DIRS} include)


add_library(autonomous_pick_place STATIC src/autonomous_pick_place.cpp)
add_library(benchmarking STATIC src/benchmarking.cpp)

add_library(grasping STATIC src/grasping.cpp)
add_library(path_planning STATIC src/path_planning.cpp)
add_library(object_detection STATIC src/object_detection.cpp)

add_executable(gripper_communication_error_solver src/gripper_communication_error_solver.cpp)
add_executable(gripper_state_adder src/gripper_state_adder.cpp)


add_executable(autonomous_pick_place_node 
              src/autonomous_pick_place_node.cpp 
              src/autonomous_pick_place.cpp)

add_executable(benchmarking_node
               src/benchmarking_node.cpp
               src/benchmarking.cpp)


add_executable(generate_data src/generate_data.cpp)
add_executable(test_node src/test_node.cpp)


target_link_libraries(test_node ${catkin_LIBRARIES})
target_link_libraries(gripper_communication_error_solver ${catkin_LIBRARIES})
target_link_libraries(gripper_state_adder ${catkin_LIBRARIES})
target_link_libraries(generate_data ${catkin_LIBRARIES})
target_link_libraries(autonomous_pick_place ${catkin_LIBRARIES})
target_link_libraries(grasping ${catkin_LIBRARIES})


target_link_libraries(autonomous_pick_place path_planning)
target_link_libraries(autonomous_pick_place grasping)
target_link_libraries(autonomous_pick_place object_detection)
target_link_libraries(autonomous_pick_place benchmarking)



target_link_libraries(autonomous_pick_place_node autonomous_pick_place)
target_link_libraries(benchmarking_node autonomous_pick_place)




install(TARGETS ${PROJECT_NAME} autonomous_pick_place grasping
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(PROGRAMS scripts/startup.sh 
scripts/head_down.sh
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS 
scripts/custom_calibration.py
scripts/publish_camera_info.py
scripts/publish_new_tf.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

