cmake_minimum_required(VERSION 3.0.2)
project(action_controllers)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  pluginlib)

# Declare catkin project
catkin_package(
    CATKIN_DEPENDS controller_interface hardware_interface
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(position_action_controller
  src/position_controller.cpp
)

target_link_libraries(position_action_controller ${catkin_LIBRARIES})


# Install
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install library
install(TARGETS position_action_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

# install(FILES position_controllers_plugins.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})