cmake_minimum_required(VERSION 2.8.3)
project(camera_v4l)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
)

find_package(Boost REQUIRED COMPONENTS
                            filesystem)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH};${PROJECT_SOURCE_DIR}/cmake)

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS}
)


add_executable(camera_v4l_node src/node.cpp)
target_link_libraries(camera_v4l_node ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS camera_v4l_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

