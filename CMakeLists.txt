cmake_minimum_required(VERSION 2.8.3)

set(PROJECTNAME "cld1a0_pcl")
project(${PROJECTNAME} CXX)


set(extra_flags -std=c++11 -Werror -Wall -Wextra)
set(CMAKE_CXX_FLGAS "${CMAKE_CXX_FLAGS} ${extra_flags}")
set(CMAKE_C_LINK_EXECUTABLE ${CMAKE_CXX_LINK_EXECUTABLE})

add_compile_options(-std=c++11)

add_definitions(${COMPILE_FLAGS} ${PCL_DEFINITIONS})
add_definitions(-Wall -O2)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  sensor_msgs
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.2 REQUIRED)

catkin_package(INCLUDE_DIRS include)
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/CygbotParser.c)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(cld1a0_pcl_publisher
    src/cld1a0_pcl_publisher.cpp
    src/cld1a0_pcl.cpp
    src/CygbotParser.c)

add_dependencies(cld1a0_pcl_publisher cld1a0_pcl_driver_generate_messages_cpp)

target_link_libraries(cld1a0_pcl_publisher
    ${PROJECT_NAME}
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES})

install(TARGETS cld1a0_pcl_publisher
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
