cmake_minimum_required(VERSION 3.8)
project(dmp_ros2)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 20)

add_subdirectory(library)

# ROS2 packages
find_package(rclcpp         REQUIRED)
find_package(geometry_msgs  REQUIRED)
find_package(tf2_ros        REQUIRED)

configure_file(
    ${CMAKE_CURRENT_LIST_DIR}/include/${PROJECT_NAME}/defines.hpp.in
    ${CMAKE_CURRENT_LIST_DIR}/include/${PROJECT_NAME}/defines.hpp
)

add_library(
    dmp_commons
    ${CMAKE_CURRENT_LIST_DIR}/src/lib/dmp_filesystem.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/lib/demonstration_handler.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/lib/preprocessing.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/lib/plots.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/lib/trajectory_info.cpp
)
target_include_directories(
    dmp_commons
    PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/private_include/dmp_filesystem
    ${CMAKE_CURRENT_LIST_DIR}/private_include/commons
    ${CMAKE_CURRENT_LIST_DIR}/include/
)
target_link_libraries(
    dmp_commons
    PUBLIC
    dmplib
    fmt::fmt
)

add_executable(
    data_preprocessor
    src/nodes/data_preprocessor.cpp
    src/entrypoints/data_preprocessor.cpp
)
target_include_directories(
    data_preprocessor
    PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)
target_link_libraries(
    data_preprocessor
    dmplib
    dmp_commons
)
ament_target_dependencies(
    data_preprocessor
    rclcpp
    geometry_msgs
    tf2_ros
)


# Goal of this executable:
# Listen for posestamped messages, store in a buffer and then write to file
add_executable(
    trajectory_listener
    src/nodes/trajectory_listener.cpp
    src/entrypoints/trajectory_listener.cpp
)
target_include_directories(
    trajectory_listener
    PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)
target_link_libraries(
    trajectory_listener
    dmplib
    dmp_commons
)
ament_target_dependencies(
    trajectory_listener
    rclcpp
    geometry_msgs
    tf2_ros
)
)



install(
    TARGETS trajectory_listener test_node data_preprocessor
    DESTINATION lib/${PROJECT_NAME}
)
install(
    DIRECTORY
    launch
    DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
