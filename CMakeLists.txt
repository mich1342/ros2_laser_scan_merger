cmake_minimum_required(VERSION 3.20)
project(ros2_laser_scan_merger)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPENDENCIES
    rclcpp
    sensor_msgs
    std_msgs
    rcl_interfaces
    rcutils
    tf2_geometry_msgs
    geometry_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Boost REQUIRED)

if("${pcl_conversions_VERSION}" VERSION_GREATER_EQUAL "2.10.0")
    list(APPEND DEPENDENCIES pcl_conversions)
endif()

foreach(Dependency IN ITEMS ${DEPENDENCIES})
    find_package(${Dependency} REQUIRED)
endforeach()

foreach(Dependency IN ITEMS ${DEPENDENCIES})
    list(APPEND INCLUDE_DEPENDS "${Dependency}::${Dependency}")
endforeach()

include_directories(
  include
  ${pcl_conversions_INCLUDE_DIRS}
)

add_executable(ros2_laser_scan_merger src/main.cpp)
target_link_libraries(ros2_laser_scan_merger ${INCLUDE_DEPENDS})
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)


install(TARGETS
ros2_laser_scan_merger
DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
