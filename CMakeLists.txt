cmake_minimum_required(VERSION 2.8.3)
project(station_map)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  rospy
  std_msgs
  geometry_msgs
  tf
  message_generation
  pcl_msgs
  )


find_package(OpenCV REQUIRED)

find_package(PCL REQUIRED)

add_message_files(
  FILES
  drone_MSG.msg
  drone_colors.msg
  drone_ver.msg
  c_msgs.msg
  e_msgs.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  pcl_msgs
)


catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include src)
include_directories(include include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS} -O3)

add_executable(st_map_node src/main.cpp include/cam_object.cpp include/ar_polygons.cpp include/update_all_polygons.cpp include/vis_errors.cpp)

add_dependencies(st_map_node station_map_generate_messages_cpp)

target_link_libraries(st_map_node
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
)
