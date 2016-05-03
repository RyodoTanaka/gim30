cmake_minimum_required(VERSION 2.8.3)
project(gim30)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp sensor_msgs
 )

# find_package(roslaunch)
# roslaunch_add_file_check(test/gim30_launch_test.xml)

include_directories(
  include
  include/urg_library-1.1.8/include
  ${catkin_INCLUDE_DIRS}
)
link_directories(
  /usr/local/lib
)

add_executable(gim30_node 
  src/gim30_node.cpp
  src/gim30_node_wrapper.cpp
)

add_library(urg_c 
  include/urg_library-1.1.8/src/urg_connection.c
  include/urg_library-1.1.8/src/urg_debug.c
  include/urg_library-1.1.8/src/urg_ring_buffer.c
  include/urg_library-1.1.8/src/urg_sensor.c
  include/urg_library-1.1.8/src/urg_serial.c
  include/urg_library-1.1.8/src/urg_serial_linux.c
  include/urg_library-1.1.8/src/urg_serial_utils.c
  include/urg_library-1.1.8/src/urg_serial_utils_linux.c
  include/urg_library-1.1.8/src/urg_tcpclient.c
  include/urg_library-1.1.8/src/urg_utils.c
)

add_dependencies(gim30_node urg_c gim30_generate_messages_cpp)

target_link_libraries(gim30_node
  urg_c
  ${catkin_LIBRARIES}
)
