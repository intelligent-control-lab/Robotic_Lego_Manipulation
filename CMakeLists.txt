cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 14)
project(lego_manipulation VERSION 1.0.0 LANGUAGES CXX)

# /* -------------------------------------------------------------------------- */
# /*                              Build Properties                              */
# /* -------------------------------------------------------------------------- */

add_compile_options(-std=c++11)
add_definitions(-D_OS_UNIX)

# /* -------------------------------------------------------------------------- */
# /*                                Find Package                                */
# /* -------------------------------------------------------------------------- */

find_package(Eigen3 3.3.7 REQUIRED)
find_package(OpenCV REQUIRED )
find_package(realsense2 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  sensor_msgs
  message_generation
  # yk_api
)

catkin_python_setup()
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS})

# /* -------------------------------------------------------------------------- */
# /*                               Create Library                               */
# /* -------------------------------------------------------------------------- */

file(GLOB src_files src/*.cpp
                    src/Utils/*.cpp
                    src/FTS/*.cpp)

add_library(${PROJECT_NAME} SHARED ${src_files})

target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
    realsense2
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    jsoncpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1)

add_service_files(
  FILES
  SetPose.srv
  GetPose.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS
  message_runtime
  # yk_api
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# /* -------------------------------------------------------------------------- */
# /*                             Testing Executables                            */
# /* -------------------------------------------------------------------------- */

add_executable(${PROJECT_NAME}_realsense
  src/ros_nodes/realsense_node.cpp
)
target_link_libraries(${PROJECT_NAME}_realsense
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_fts
  src/ros_nodes/fts_node.cpp
)
target_link_libraries(${PROJECT_NAME}_fts
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_task_planning
  src/ros_nodes/lego_task_planning_node.cpp
)
add_dependencies(${PROJECT_NAME}_task_planning
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_task_planning
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})