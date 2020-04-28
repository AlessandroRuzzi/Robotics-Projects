cmake_minimum_required(VERSION 2.8.3)
project(first_project)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geodesy sensor_msgs tf nav_msgs message_filters message_generation dynamic_reconfigure)

generate_dynamic_reconfigure_options(
  cfg/flag.cfg
  
)

add_service_files(
  FILES
  distance_calculate.srv
)

add_message_files(
 FILES
 dist.msg

 )


generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(converter src/converter.cpp
	       src/converter.cpp include/converter.h)
target_link_libraries(converter ${catkin_LIBRARIES})

add_executable(distance_publisher src/distance_publisher.cpp
		src/distance_publisher.cpp include/distance_publisher.h)
add_dependencies(distance_publisher ${${PROJECT_NAME}_EXPORTE_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(distance_publisher custom_messages_generate_messages_cpp)
add_dependencies(distance_publisher ${PROJECT_NAME}_generate_messages_cpp )
add_dependencies(distance_publisher ${PROJECT_NAME}_gencfg)

target_link_libraries(distance_publisher ${catkin_LIBRARIES})


add_executable(distance_calculator src/distance_calculator.cpp
		)
add_dependencies(distance_calculator ${${PROJECT_NAME}_EXPORTE_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(distance_calculator ${PROJECT_NAME}_generate_messages_cpp )
target_link_libraries(distance_calculator ${catkin_LIBRARIES})


