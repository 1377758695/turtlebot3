cmake_minimum_required(VERSION 2.8.3)
project(navigation)

find_package(catkin REQUIRED COMPONENTS actionlib move_base_msgs roscpp tf std_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

# Build the executable
add_executable(get_to_drug_shelf src/get_to_drug_shelf.cpp)
add_executable(get_to_destination src/get_to_destination.cpp)
add_executable(get_to_destination_test src/get_to_destination_2.cpp)
add_executable(return_to_start src/return_to_start.cpp)

# Add a build order dependency on nav_msgs
# This ensures that all msg headers are built before your executable
if(catkin_EXPORTED_TARGETS)
  add_dependencies(get_to_drug_shelf ${catkin_EXPORTED_TARGETS})
  add_dependencies(get_to_destination ${catkin_EXPORTED_TARGETS})  
  add_dependencies(get_to_destination_test ${catkin_EXPORTED_TARGETS})  
  add_dependencies(return_to_start ${catkin_EXPORTED_TARGETS})
endif()
# Link against the catkin libraries
target_link_libraries(get_to_drug_shelf ${catkin_LIBRARIES})
target_link_libraries(get_to_destination ${catkin_LIBRARIES})
target_link_libraries(get_to_destination_test ${catkin_LIBRARIES})
target_link_libraries(return_to_start ${catkin_LIBRARIES})

