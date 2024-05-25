cmake_minimum_required(VERSION 3.10.2)
project(depthai_ros_my VERSION 2.9.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

#add_compile_options(-g)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  #add_compile_options(-Wall -Wextra -Wpedantic -DDEPTHAI_HAVE_OPENCV_SUPPORT)
endif()

set(_opencv_version 4)
find_package(OpenCV 4 QUIET COMPONENTS imgproc highgui)
if(NOT OpenCV_FOUND)
  set(_opencv_version 3)
  find_package(OpenCV 3 REQUIRED COMPONENTS imgproc highgui)
endif()

#------ insert download ML model start ------- 
# find_package(depthai CONFIG REQUIRED PATHS "/home/sachin/Desktop/luxonis/depthai-core/build/install/lib/cmake/depthai")
set(tiny_yolo_v4_blob_name "yolov4_tiny_coco_416x416_openvino_2021.4_6shave_bgr.blob")
set(mobilenet_blob_name "mobilenet-ssd_openvino_2021.2_6shave.blob")
set(deeplab_blob_name "deeplab_v3_plus_mnv2_decoder_256_openvino_2021.4.blob")

set(mobilenet_blob "${PROJECT_SOURCE_DIR}/resources/${mobilenet_blob_name}")
file(DOWNLOAD "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/mobilenet-ssd_openvino_2021.2_6shave.blob"
${mobilenet_blob}
TIMEOUT 60  # seconds
EXPECTED_HASH SHA1=f0e14978b3f77a4f93b9f969cd39e58bb7aef490
TLS_VERIFY ON)

set(tiny_yolo_v4_blob "${PROJECT_SOURCE_DIR}/resources/${tiny_yolo_v4_blob_name}")
file(DOWNLOAD "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/tiny-yolo-v4_openvino_2021.2_6shave.blob"
  ${tiny_yolo_v4_blob}
  TIMEOUT 60  # seconds
  EXPECTED_HASH SHA1=219d949610a5760e62a8458941e1300b81c3fe4a
  TLS_VERIFY ON)

set(deeplab_blob "${PROJECT_SOURCE_DIR}/resources/${deeplab_blob_name}")
file(DOWNLOAD
"https://github.com/luxonis/depthai-experiments/blob/master/gen2-deeplabv3_multiclass/models/deeplab_v3_plus_mnv2_decoder_256_openvino_2021.4.blob?raw=true"
  ${deeplab_blob}
  TIMEOUT 60  # seconds
  EXPECTED_HASH SHA1=bdc2529f765f34c4799b478690fb4e32f1ad5d89
  TLS_VERIFY ON)
#------inssert download ML model end  ------- 

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
  src/detect_com.cpp
  src/utility.cpp
  )

  target_link_libraries("${node_name}"
                        depthai::core
                        opencv_imgproc
                        opencv_highgui)

  ament_target_dependencies("${node_name}"
                            ${dependencies})

endmacro()

dai_add_node_ros2(detect_subscriber src/detect_subscriber.cpp)
#
dai_add_node_ros2(rgb_stereo_node src/rgb_stereo_node.cpp)
dai_add_node_ros2(stereo_publisher src/stereo_publisher.cpp)
# add by nishi 2024.5.10
dai_add_node_ros2(stereo_camera_test src/stereo_camera_test.cpp)
dai_add_node_ros2(stereo_camera_test2 src/stereo_camera_test2.cpp)
dai_add_node_ros2(stereo_publisher_my src/stereo_publisher_my.cpp)
dai_add_node_ros2(rgb_stereo_node_my src/rgb_stereo_node_my.cpp)
dai_add_node_ros2(mobilenet_publisher src/mobilenet_publisher.cpp)
#
dai_add_node_ros2(rgb_mobilenet src/rgb_mobilenet.cpp)
dai_add_node_ros2(tiny_yolo_v4 src/tiny_yolo.cpp)
#


target_compile_definitions(mobilenet_publisher PRIVATE BLOB_NAME="${mobilenet_blob_name}")
target_compile_definitions(rgb_mobilenet PRIVATE BLOB_PATH="${mobilenet_blob_name}")
target_compile_definitions(tiny_yolo_v4 PRIVATE BLOB_PATH="${tiny_yolo_v4_blob_name}")

if($ENV{ROS_DISTRO} STREQUAL "galactic")
  target_compile_definitions(rgb_stereo_node PRIVATE IS_GALACTIC)
endif()

#target_compile_definitions(stereo_camera_test PUBLIC DEPTHAI_HAVE_OPENCV_SUPPORT)


install(TARGETS
        detect_subscriber
        rgb_stereo_node
        stereo_publisher
        stereo_camera_test
        stereo_camera_test2
        rgb_stereo_node_my
        stereo_publisher_my
        mobilenet_publisher
        rgb_mobilenet
        tiny_yolo_v4
        DESTINATION lib/${PROJECT_NAME})

# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY resources DESTINATION share/${PROJECT_NAME})


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
