cmake_minimum_required(VERSION 2.8.3)
project(youbot_moveit_config)

find_package(catkin REQUIRED 
youbot_driver
moveit_ros_planning_interface
)
catkin_package(CATKIN_DEPENDS soem)
include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_definitions(-DCONFIG_FOLDER_PATH="/home/jp/catkin_ws/src/youbot_driver/config/")
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(youbot_moveit_config src/youbot_moveit_config.cpp)
target_link_libraries(youbot_moveit_config ${catkin_LIBRARIES} ${Boost_LIBRARIES})

