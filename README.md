### depthai_ros_my  
OAK-D Lite 対応 ROS2 Humble depthai-ros example の改造版です。  

  ubuntu 22.04  
  PC and OrangePi 5 (Armbian and ubuntu 22.04)  
  ros2:humble  

オリジナル: [depthai-ros / depthai_examples](https://github.com/luxonis/depthai-ros/tree/humble/depthai_examples)    

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

    1) publish img_rect and depth for ratbmap_ros stereo rect  
    $ ros2 launch depthai_ros_my oak-d_stereo.launch.py    
    rem. build and exec src/rgb_stereo_node_my.cpp  

    2) publish rgb and depth for rtambap depth   
    $ ros2 launch depthai_ros_my oak-d_rgb_stereo_node.launch.py  
    rem. build and exec src/rgb_stereo_node_my.cpp  
   
#### 4. my library  

    src/camera_com.cpp and include/depthai_ros_my/camera_com.hpp    
    It is very simple and easy than depthai-ros/depthai_bridge/src/ImageConverter.cpp, I think.
