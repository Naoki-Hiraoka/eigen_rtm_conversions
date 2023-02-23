cmake_minimum_required(VERSION 3.0.2)
project(eigen_rtm_conversions)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN eigen3>=3.2.7)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES eigen_rtm_conversions
)

include_directories(
  include
  ${EIGEN_INCLUDE_DIRS}
  ${openrtm_aist_INCLUDE_DIRS}
  )

add_library(${PROJECT_NAME} SHARED src/eigen_rtm_conversions.cpp)
target_link_directories(${PROJECT_NAME} PUBLIC
  ${openrtm_aist_LIBRARY_DIRS})
target_link_libraries(${PROJECT_NAME}
  ${openrtm_aist_LIBRARIES}
  ${EIGEN_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  )
