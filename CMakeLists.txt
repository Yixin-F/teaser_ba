cmake_minimum_required(VERSION 3.0.2)
project(teaser_ba)

option(ENABLE_TEASER "Build with Teaser++" TRUE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  geometry_msgs
  message_generation
)

find_package(Ceres REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# include(ExternalProject)
# if(ENABLE_TEASER)
#   ExternalProject_Add(teaser
#     GIT_REPOSITORY https://github.com/koide3/TEASER-plusplus
#     GIT_TAG master
#     BUILD_IN_SOURCE
#     CMAKE_ARGS -DBUILD_TESTS=OFF -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_DOC=OFF -DBUILD_TEASER_FPFH=ON -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
#     INSTALL_COMMAND make install
#   )

#   set(TEASER_LIBRARIES
#     ${CATKIN_DEVEL_PREFIX}/lib/libpmc.so
#     ${CATKIN_DEVEL_PREFIX}/lib/libteaser_registration.so
#     ${CATKIN_DEVEL_PREFIX}/lib/libteaser_features.so
#   )

#   add_definitions(-DTEASER_ENABLED)
# endif()

set(TEASER_LIBRARIES
  /home/yixin/teaser_ba/src/teaser_ba/thirdparty/teaser/libpmc.so
  /home/yixin/teaser_ba/src/teaser_ba/thirdparty/teaser/libteaser_registration.so
  /home/yixin/teaser_ba/src/teaser_ba/thirdparty/teaser/libteaser_features.so
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

add_library(teaser_head include/teaser-toolkit/TeaserHead.cc)
target_link_libraries(teaser_head ${TEASER_LIBRARIES})

add_library(ceres_head include/ceres/PointToPoint.cc include/ceres/pose_local_parameterization.cc)
target_link_libraries(ceres_head ${CERES_LIBRARIES})


add_executable(main_node src/main_node.cc)
target_link_libraries(main_node
    teaser_head
    ceres_head
    glog
    OpenMP::OpenMP_CXX
    ${catkin_LIBRARIES}
)

add_executable(test_node src/test_node.cc)
target_link_libraries(test_node
    teaser_head
    ceres_head
    glog
    OpenMP::OpenMP_CXX
    ${catkin_LIBRARIES}
)