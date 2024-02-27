cmake_minimum_required(VERSION 3.5.1)
project(pointcloud_io)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Options.
option(BUILD_WITH_VTK_SUPPORT "Build package with support for VTK files." ON)
if(BUILD_WITH_VTK_SUPPORT)
  message("pointcloud_io will be built with support for VTK files.")
  add_definitions(-DHAVE_VTK)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

ament_export_dependencies(pcl_ros rclcpp sensor_msgs shape_msgs visualization_msgs)
ament_export_include_directories(include)

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
)


################
## Define MSG ##
################
rosidl_generate_interfaces(${PROJECT_NAME}_msgs
  "msg/TextureMesh.msg"
  "srv/GetMesh.srv"
  "srv/GetTextureMesh.srv"
  DEPENDENCIES geometry_msgs visualization_msgs sensor_msgs shape_msgs std_msgs
  ADD_LINTER_TESTS
  LIBRARY_NAME ${PROJECT_NAME}
)
ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}_msgs rosidl_typesupport_cpp)


###########
## Build ##
###########
add_executable(read
  src/read_node.cpp
  src/Read.cpp
)
ament_target_dependencies(read
  rclcpp pcl_ros sensor_msgs shape_msgs)
target_link_libraries(read "${cpp_typesupport_target}")

add_executable(write
  src/write_node.cpp
  src/Write.cpp
)
ament_target_dependencies(write rclcpp pcl_ros sensor_msgs)


#############
## Install ##
#############
install(
  TARGETS
    read
    write
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories(include)
install(
  DIRECTORY
    examples
    launch
  DESTINATION share/${PROJECT_NAME}
)

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling(
    TARGETS
      read
      write
  )
endif(cmake_clang_tools_FOUND)

##############
##   Test   ##
##############
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME}
    test/empty_test.cpp
  )
  add_dependencies(test_${PROJECT_NAME}
    read
    write
  )
  target_include_directories(test_${PROJECT_NAME} PRIVATE
    include
  )
  target_include_directories(test_${PROJECT_NAME} SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(test_${PROJECT_NAME}
    gtest_main
    ${catkin_LIBRARIES}
  )

  ###################
  ## Code_coverage ##
  ###################
  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage(
      TEST_BUILD_TARGETS
        test_${PROJECT_NAME}
    )
  endif()
endif()


ament_package()
