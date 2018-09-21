cmake_minimum_required(VERSION 2.8.3)
project(odom_predictor)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=gnu++11 -o3)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/odom_predictor.cpp
)

############
# BINARIES #
############
cs_add_executable(odom_predictor_node
  src/odom_predictor_node.cpp
)
target_link_libraries(odom_predictor_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
