/*
* depthai_ros_my/src/detect_com.cpp
*/

#include "depthai_ros_my/camera_com.hpp"

//#include "sensor_msgs/msg/image.hpp"
//#include "vision_msgs/msg/detection2_d_array.hpp"

//#include <vision_msgs/msg/detection2_d_array.hpp>


namespace camera_com {

//namespace VisionMsgs = vision_msgs::msg;
//using Detection2DArrayPtr = VisionMsgs::Detection2DArray::SharedPtr;


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
void Go_DtectPublish::openPub_noInfo(std::shared_ptr<rclcpp::Node> node, std::string frame_name, std::string topic_name, int qos,int width, int height , bool normalized){

    if(debug_f_)
        std::cout << "openPub_noInfo() start" << std::endl;

    node_=node;
    frame_name_ = frame_name;
    topic_name_ = topic_name;
    width_=width;
    height_=height;
    normalized_=normalized;

    Que_Recv::pub_ready_=true;

    //#define USE_ORG_QOS
    #if defined(USE_ORG_QOS)
        int video_qos = qos;
    #else
        rclcpp::QoS video_qos(1);
        video_qos.reliability((rmw_qos_reliability_policy_t)qos);
    #endif

    //auto detect_pub_ = node_->create_publisher<vision_msgs::msg::Detection2DArray>(topic_name_, video_qos);
    detect_pub_ = node_->create_publisher<vision_msgs::msg::Detection2DArray>(topic_name_, video_qos);

    //printf("%s",detect_pub_);

    noInfo_f_=true;

    if(debug_f_)
        std::cout << "openPub_noInfo(): 99" << std::endl;

}

/*--------------
* Go_DtectPublish::feedImages()
* reffer from
*  /home/nishi/colcon_ws/src/depthai-ros/depthai_bridge/src/ImgDetectionConverter.cpp
*   void ImgDetectionConverter::toRosMsg(std::shared_ptr<dai::ImgDetections> inNetData, std::deque<VisionMsgs::Detection2DArray>& opDetectionMsgs)
*
* call sequence
* 1) depthai-ros/depthai_bridge/include/depthai_bridge/BridgePublisher.hpp
*  line 219     _daiMessageQueue->addCallback(std::bind(&BridgePublisher<RosMsg, SimMsg>::daiCallback, this, std::placeholders::_1, std::placeholders::_2));
*   ↓
*  line 181  void BridgePublisher<RosMsg, SimMsg>::daiCallback(std::string name, std::shared_ptr<ADatatype> data)
*   ↓
*  line 224 void BridgePublisher<RosMsg, SimMsg>::publishHelper(std::shared_ptr<SimMsg> inDataPtr)
*   ↓
* 2) depthai-ros/depthai_bridge/src/ImgDetectionConverter.cpp
*   line 25 
*      void ImgDetectionConverter::toRosMsg(std::shared_ptr<dai::ImgDetections> inNetData, std::deque<VisionMsgs::Detection2DArray>& opDetectionMsgs) 
*      & opDetectionMsgs に、データを変換する。
* 
----------------*/
//void Go_DtectPublish::feedImages(std::shared_ptr<dai::ImgFrame> &inData){
void Go_DtectPublish::feedImages(std::shared_ptr<dai::ADatatype> &Data){

    if(debug_f_){
        std::cout << "feedImagesPub() name:=" << Que_Recv::name_ << std::endl;
        std::cout << " frame_name_:"<< frame_name_ << std::endl;
    }
    //auto inData = std::dynamic_pointer_cast<dai::ImgFrame>(Data);
    auto inNetData = std::dynamic_pointer_cast<dai::ImgDetections>(Data);

    #if defined(USE_ORG_TIME)
        // reffer from
        // depthai-ros/depthai_bridge/src/ImgDetectionConverter.cpp
        //  line 25 void ImgDetectionConverter::toRosMsg()
        if(_updateRosBaseTimeOnToRosMsg) {
            updateRosBaseTime();
        }
        std::chrono::_V2::steady_clock::time_point tstamp;
        if(_getBaseDeviceTimestamp)
            tstamp = inNetData->getTimestampDevice();
        else
            tstamp = inNetData->getTimestamp();
    #else
        rclcpp::Time capture_time = node_->now();
    #endif

    if(trace_){
        cnt_++;
        rclcpp::Duration elapsed = capture_time - start_t_;
        if(elapsed.seconds() >= sec_dur_){
            std::cout << " oak-d_detect: "<< Que_Recv::name_ << " rate: " << cnt_/(int)sec_dur_ << std::endl;
            start_t_ = capture_time;
            cnt_=0;
        }
    }

    //VisionMsgs::Detection2DArray opDetectionMsg;
    vision_msgs::msg::Detection2DArray opDetectionMsg;

    //opDetectionMsg.header.stamp = getFrameTime(_rosBaseTime, _steadyBaseTime, tstamp);
    opDetectionMsg.header.stamp = capture_time;
    opDetectionMsg.header.frame_id = frame_name_;
    opDetectionMsg.detections.resize(inNetData->detections.size());

    if(debug_f_){
        std::cout << " inNetData->detections.size():"<< inNetData->detections.size() << std::endl;
    }

    for(int i = 0; i < inNetData->detections.size(); ++i) {

        float xMin, yMin, xMax, yMax;
        xMin = inNetData->detections[i].xmin;
        yMin = inNetData->detections[i].ymin;
        xMax = inNetData->detections[i].xmax;
        yMax = inNetData->detections[i].ymax;

        #if defined(DEBUG_CHK_11)
            // yolo v4 だと、結構、超えるみたいだ。
            if( xMin < 0 || xMin > 1.0)
                std::cout << " xMin:"<< xMin << std::endl;
            if( xMax < 0 || xMax > 1.0)
                std::cout << " xMax:"<< xMax << std::endl;
            if( yMin < 0 || yMin > 1.0)
                std::cout << " yMin:"<< yMin << std::endl;
            if( yMax < 0 || yMax > 1.0)
                std::cout << " yMax:"<< yMax << std::endl;
        #endif

        if(!normalized_) {
            xMin *= (float)width_;
            yMin *= (float)height_;
            xMax *= (float)width_;
            yMax *= (float)height_;
        }

        float xSize = xMax - xMin;
        float ySize = yMax - yMin;
        float xCenter = xMin + xSize * 0.5;
        float yCenter = yMin + ySize * 0.5;

        opDetectionMsg.detections[i].results.resize(1);

        #define IS_HUMBLE
        #if defined(IS_GALACTIC) || defined(IS_HUMBLE)
            //opDetectionMsg.detections[i].id = std::to_string(inNetData->detections[i].label);
            uint32_t labelIndex = inNetData->detections[i].label;
            std::string labelStr = std::to_string(labelIndex);
            if(is_labelMap_){
                // changed by nishi 2024.5.24
                if(labelIndex < labelMap_->size()){
                    labelStr = labelMap_->at(labelIndex);
                }
            }
            opDetectionMsg.detections[i].id = labelStr;
            opDetectionMsg.detections[i].results[0].hypothesis.class_id = std::to_string(inNetData->detections[i].label);
            opDetectionMsg.detections[i].results[0].hypothesis.score = inNetData->detections[i].confidence;
        #elif defined(IS_ROS2)
            opDetectionMsg.detections[i].results[0].id = std::to_string(inNetData->detections[i].label);
            opDetectionMsg.detections[i].results[0].score = inNetData->detections[i].confidence;
        #endif
        #if defined(IS_HUMBLE)
            opDetectionMsg.detections[i].bbox.center.position.x = xCenter;
            opDetectionMsg.detections[i].bbox.center.position.y = yCenter;
        #else
            opDetectionMsg.detections[i].bbox.center.x = xCenter;
            opDetectionMsg.detections[i].bbox.center.y = yCenter;
        #endif
        opDetectionMsg.detections[i].bbox.size_x = xSize;
        opDetectionMsg.detections[i].bbox.size_y = ySize;
    }

    // exe publish
    detect_pub_->publish(opDetectionMsg);

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