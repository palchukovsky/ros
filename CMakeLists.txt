cmake_minimum_required(VERSION 2.8.3)
project(demo)

add_compile_options(-Wall -pedantic -std=c++17 -Wfatal-errors)

find_package(catkin REQUIRED
  autoware_build_flags
  roscpp
  rospy
  rosunit
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pub src/Publisher.cpp)
add_dependencies(pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pub ${catkin_LIBRARIES})
install(TARGETS pub
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(sub src/Subscriber.cpp)
add_dependencies(sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sub ${catkin_LIBRARIES})
install(TARGETS sub
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
foreach(T
    scripts)
  install(DIRECTORY ${T}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS)
endforeach()