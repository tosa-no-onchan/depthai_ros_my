cmake_minimum_required(VERSION 3.10.2)
project(depthai_ros_my VERSION 2.9.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  #add_compile_options(-Wall -Wextra -Wpedantic -DDEPTHAI_HAVE_OPENCV_SUPPORT)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(depthai CONFIG REQUIRED)
find_package(depthai_ros_msgs REQUIRED)
find_package(depthai_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

set(dependencies
  camera_info_manager
  cv_bridge
  depthai_ros_msgs
  depthai_bridge
  rclcpp
  sensor_msgs
  stereo_msgs
  std_msgs
  vision_msgs
)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

macro(dai_add_node_ros2 node_name node_src)
  add_executable("${node_name}" "${node_src}"
  src/camera_com.cpp
  )

  target_link_libraries("${node_name}"
                        depthai::core
                        opencv_imgproc
                        opencv_highgui)

  ament_target_dependencies("${node_name}"
                            ${dependencies})

endmacro()

dai_add_node_ros2(rgb_stereo_node src/rgb_stereo_node.cpp)
dai_add_node_ros2(stereo_publisher src/stereo_publisher.cpp)
# add by nishi 2024.5.10
dai_add_node_ros2(stereo_camera_test src/stereo_camera_test.cpp)
dai_add_node_ros2(stereo_camera_test2 src/stereo_camera_test2.cpp)
dai_add_node_ros2(stereo_publisher_my src/stereo_publisher_my.cpp)
dai_add_node_ros2(rgb_stereo_node_my src/rgb_stereo_node_my.cpp)

if($ENV{ROS_DISTRO} STREQUAL "galactic")
  target_compile_definitions(rgb_stereo_node PRIVATE IS_GALACTIC)
endif()

#target_compile_definitions(stereo_camera_test PUBLIC DEPTHAI_HAVE_OPENCV_SUPPORT)


install(TARGETS
        rgb_stereo_node
        stereo_publisher
        stereo_camera_test
        stereo_camera_test2
        rgb_stereo_node_my
        stereo_publisher_my
        DESTINATION lib/${PROJECT_NAME})

# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
