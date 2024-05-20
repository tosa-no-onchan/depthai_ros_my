/*
* depthai_ros_my/src/detect_com.cpp
*/

#include "depthai_ros_my/camera_com.hpp"

namespace camera_com {

/*
* classs Go_DtectPublish members
*/
/*--------------
* Go_DtectPublish::openPub()
----------------*/
void Go_DtectPublish::openPub(std::shared_ptr<rclcpp::Node> node, std::string frame_name, std::string topic_name, int qos, sensor_msgs::msg::CameraInfo &cameraInfo,bool trace){
    node_=node;
    frame_name_ = frame_name;
    topic_name_ = topic_name;
    trace_ = trace; 

    if(debug_f_)
        std::cout << "openPub() start" << std::endl;

    info_mgr_ = std::make_shared<camera_info_manager::CameraInfoManager>(node_.get());
    info_mgr_->setCameraInfo(cameraInfo);

    //#define TEST_KKX
    //#if defined(TEST_KKX)
    rmw_qos_profile_t video_qos_profile = rmw_qos_profile_sensor_data;
    video_qos_profile.depth=1;
    switch(qos){
        case 0:
            video_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
        break;
        case 1:
            video_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        break;
        case 2:
            video_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        break;
    }
    //video_qos_profile.reliability = (rmw_qos_reliability_policy_t)qos;

    //video_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    video_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;

    camera_transport_pub_ = image_transport::create_camera_publisher(node_.get(), topic_name_, video_qos_profile);

    //#endif

    // rate trace ON ?
    if(trace_){
        start_t_ = node_->now();
    }

    Que_Recv::pub_ready_=true;

    if(debug_f_)
        std::cout << "openPub(): 99" << std::endl;
}

/*--------------
* Go_DtectPublish::openPub_noInfo()
----------------*/
void Go_DtectPublish::openPub_noInfo(std::shared_ptr<rclcpp::Node> node, std::string frame_name, std::string topic_name, int qos){

    node_=node;
    frame_name_ = frame_name;
    topic_name_ = topic_name;

    Que_Recv::pub_ready_=true;

    noInfo_f_=true;

}

/*--------------
* Go_DtectPublish::feedImages()
* reffer from
*  /home/nishi/colcon_ws/src/depthai-ros/depthai_bridge/src/ImgDetectionConverter.cpp
*   void ImgDetectionConverter::toRosMsg(std::shared_ptr<dai::ImgDetections> inNetData, std::deque<VisionMsgs::Detection2DArray>& opDetectionMsgs)
----------------*/
//void Go_DtectPublish::feedImages(std::shared_ptr<dai::ImgFrame> &inData){
void Go_DtectPublish::feedImages(std::shared_ptr<dai::ADatatype> &Data){

    if(debug_f_){
        std::cout << "feedImagesPub() name:=" << Que_Recv::name_ << std::endl;
        std::cout << " frame_name_:"<< frame_name_ << std::endl;
    }
    auto inData = std::dynamic_pointer_cast<dai::ImgFrame>(Data);
    //auto inData = std::dynamic_pointer_cast<dai::ImgDetections>(Data);
    rclcpp::Time capture_time = node_->now();

    //VisionMsgs::Detection2DArray opDetectionMsg;


    if(trace_){
        cnt_++;
        rclcpp::Duration elapsed = capture_time - start_t_;
        if(elapsed.seconds() >= sec_dur_){
            std::cout << " oak-d_detect: "<< Que_Recv::name_ << " rate: " << cnt_/(int)sec_dur_ << std::endl;
            start_t_ = capture_time;
            cnt_=0;
        }
    }


    if(debug_f_){
        std::cout << " frame_name_:"<< frame_name_ << std::endl;
        //std::cout << " image->header.frame_id:"<< image->header.frame_id << std::endl;
    }
    //if(noInfo_f_!=true)
    //    sendInfo(capture_time, image);
}

/*--------------
* Go_DtectPublish::sendInfo()
----------------*/
void Go_DtectPublish::sendInfo(rclcpp::Time time, std::shared_ptr<sensor_msgs::msg::Image> const & img) {

    if(debug_f_)
        std::cout << " called Go_Publish::sendInfo()" << std::endl;

    std::shared_ptr<sensor_msgs::msg::CameraInfo> info = std::make_shared<sensor_msgs::msg::CameraInfo>(info_mgr_->getCameraInfo());
    //printf("%s",info_left);

    if (!checkCameraInfo(*img, *info)) {
        *info = sensor_msgs::msg::CameraInfo{};
        info->height = img->height;
        info->width = img->width;
    }

    info->header.frame_id = frame_name_;
    info->header.stamp = time;
    // add by nishi 2024.5.4
    //img->header.stamp = time;

    //trace_sts_=20;

    //if(intra_){
    //    left_pub_->publish(*img_l);
    //    left_info_pub_->publish(*info_left);
    //}
    //else{
        // http://docs.ros.org/en/jade/api/image_transport/html/classimage__transport_1_1CameraPublisher.html
        // https://github.com/ros-perception/image_transport_tutorials
        camera_transport_pub_.publish(img, info);

    //trace_sts_=23;
    //}
}

}