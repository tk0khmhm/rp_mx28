cmake_minimum_required(VERSION 2.8.3)
project(rp_mx28)

#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -O2")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  std_msgs
  roscpp
  tf
  tf2
)

find_package(Eigen3 REQUIRED)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
	message("OPENMP FOUND")
	set(MAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES rp_mx28
  CATKIN_DEPENDS geometry_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a cpp library
# add_library(rp_mx28
#   src/${PROJECT_NAME}/rp_mx28.cpp
# )

## Declare a cpp executable
# add_executable(rp_mx28_node src/rp_mx28_node.cpp)
add_executable(mx28_test src/mx28_test.cpp)
target_link_libraries(mx28_test
  ${catkin_LIBRARIES}
  wiringPi
)

add_executable(mx28 src/mx28.cpp)
target_link_libraries(mx28
  ${catkin_LIBRARIES}
  wiringPi
)

add_executable(mx28_interrupt src/mx28_interrupt.cpp)
target_link_libraries(mx28_interrupt
  ${catkin_LIBRARIES}
  wiringPi
)

add_executable(mx28_interrupt_baud src/mx28_interrupt_baud.cpp)
target_link_libraries(mx28_interrupt_baud
  ${catkin_LIBRARIES}
  wiringPi
)

add_executable(mx28_kaizoutyuu src/mx28_kaizoutyuu.cpp)
target_link_libraries(mx28_kaizoutyuu
  ${catkin_LIBRARIES}
  wiringPi
)

add_executable(mx28_error src/mx28_error.cpp)
target_link_libraries(mx28_error
  ${catkin_LIBRARIES}
  wiringPi
)

add_executable(conical_high_speed src/conical_high_speed.cpp)
target_link_libraries(conical_high_speed
  ${catkin_LIBRARIES}
)

add_executable(conical_high_speed_eth src/conical_high_speed_eth.cpp)
target_link_libraries(conical_high_speed_eth
  ${catkin_LIBRARIES}
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(rp_mx28_node rp_mx28_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(rp_mx28_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS rp_mx28 rp_mx28_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rp_mx28.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
