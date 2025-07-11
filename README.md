### depthai_ros_my  
OAK-D Lite 対応 ROS2 Humble depthai-ros example の改造版です。  

    stereo_publisher_my.cpp  
    rgb_stereo_node_my.cpp  
    stereo_camera_test2.cpp  

  ubuntu 24.04  
  PC and OrangePi 5 (Armbian and ubuntu 24.04)  
  ros2:jazzy  

オリジナル: [depthai-ros / depthai_examples](https://github.com/luxonis/depthai-ros/tree/humble/depthai_examples)    

    stereo_publisher.cpp  
    rgb_stereo_node.cpp  

#### 1. down load depthai-ros and build.  
参照: [Install from source](https://docs-beta.luxonis.com/software/ros/depthai-ros/build/)  

    $ cd ~/colcon_ws/src  
    $ git clone -b humble https://github.com/luxonis/depthai-ros.git  
    $ cd ..  
    $ rosdep update --rosdistro=humble && rossdep install --from-path src/depthai-ros --ignore-src -r [-s]  
      -s : sumilate option  
    $ source /opt/ros/humble/setup.bash  
    $ rm -rf build  
    $ rm -rf install  
    $ colcon build --symlink-install [--parallel-workers 1]   
    $ . install/setup.bash  

#### 2. down load depthai_ros_my and build.  

    $ cd ~/colcon_ws/src  
    $ git clone -b humble humble https://github.com/tosa-no-onchan/depthai_ros_my.git  
    $ cd ..  
    $ colcon build --symlink-install [--parallel-workers 1] --packages-select depthai_ros_my  
    $ . install/setup.bash  

#### 3. 実行  

i. publish img_rect and depth for ratbmap_ros stereo rect  

    $ ros2 launch depthai_ros_my oak-d_stereo.launch.py    

rem. exec src/stereo_publisher_my.cpp  

ii. publish rgb and depth for rtambap_ros depth  

    $ ros2 launch depthai_ros_my oak-d_rgb_stereo_node.launch.py  

rem. exec src/rgb_stereo_node_my.cpp  
Use USB3.x cable and sufficiant power supply (USB3.x Port).  

iii. publish mono8 stereo  

    $ ros2 launch depthai_ros_my stereo_camera.launch.py  

rem. exec src/stereo_camera_test2.cpp  

iv. publish and subscribe mobilnet object detection  
Use USB3.x cable and sufficiant power supply (USB3.x Port).  

    term1  
    $ ros2 launch depthai_ros_my mobilenet_publisher.launch.py  
    check acutual topic output.  
    $ ros2 topic echo /color/image  

    term2  
    $ ros2 run depthai_ros_my detect_subscriber  

#### 4. My library  

    src/camera_com.cpp and include/depthai_ros_my/camera_com.hpp    

It is very simple and easy than depthai-ros/depthai_bridge/src/ImageConverter.cpp, I think. 
  
example code  

    src/stereo_publisher_my.cpp  

How to make camera info.  
````
    camera_com::CameraTools camera_tools;
    camera_tools._reverseStereoSocketOrder = true;   // rtabmap_ros の rect_img のときは必要。
    auto leftCameraInfo = camera_tools.calibrationToCameraInfo_my(calibrationHandler, dai::CameraBoardSocket::CAM_B, monoWidth, monoHeight);
    auto rightCameraInfo = camera_tools.calibrationToCameraInfo_my(calibrationHandler, dai::CameraBoardSocket::CAM_C, monoWidth, monoHeight);

````

How to get images from oak-d Lite depthai camera and publish them.  
````
    // if文の{}の中に置くと、うまく動きません。必ず直において下さい、
    camera_com::Go_Publish go_pub_left,go_pub_right;

    go_pub_left.init(leftQueue);
    //go_pub_left.set_debug();
    go_pub_left.openPub(node, tfPrefix + "_left_camera_optical_frame", "left/image_rect", qos, leftCameraInfo);
    go_pub_right.init(rightQueue);
    go_pub_right.openPub(node, tfPrefix + "_right_camera_optical_frame", "right/image_rect", qos, rightCameraInfo);

    rclcpp::spin(node);
````
#### 5.Append  

    Ros2 oak-d lite runs stereo depth and mobilenet objectdetection.  
    rgb_stereo_mobilenet_node_my.cpp  
    launch file  
    oak-d_rgb_stereo_mobilenet_node.launch.py  
    
    
