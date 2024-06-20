/*
* depthai_ros_my/src/disparity_com.cpp
*/

#include "depthai_ros_my/camera_com.hpp"

//#include "sensor_msgs/msg/image.hpp"
//#include "vision_msgs/msg/detection2_d_array.hpp"

//#include <vision_msgs/msg/detection2_d_array.hpp>


namespace camera_com {

//namespace VisionMsgs = vision_msgs::msg;
//using Detection2DArrayPtr = VisionMsgs::Detection2DArray::SharedPtr;

/*--------------
* Go_Disparity::openPub()
----------------*/
void Go_Disparity::openPub(std::shared_ptr<rclcpp::Node> node, std::string frame_name, std::string topic_name,std::string info_topic_name, int qos, sensor_msgs::msg::CameraInfo &cameraInfo,bool trace){
    node_=node;
    frame_name_ = frame_name;
    topic_name_ = topic_name;
    info_topic_name_=info_topic_name;
    trace_ = trace; 

    if(debug_f_)
        std::cout << "Go_Disparity::openPub() start" << std::endl;

    //info_mgr_ = std::make_shared<camera_info_manager::CameraInfoManager>(node_.get());
    //info_mgr_->setCameraInfo(cameraInfo);
    //cameraInfo_=cameraInfo;

    info_= std::make_shared<sensor_msgs::msg::CameraInfo>(cameraInfo);



    //#define TEST_KKX
    //#if defined(TEST_KKX)
    //rmw_qos_profile_t video_qos_profile = rmw_qos_profile_sensor_data;
    //video_qos_profile.depth=1;
    //switch(qos){
    //    case 0:
    //        video_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
    //    break;
    //    case 1:
    //        video_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    //    break;
    //    case 2:
    //        video_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    //    break;
    //}
    //video_qos_profile.reliability = (rmw_qos_reliability_policy_t)qos;

    //video_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    //video_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;

    //camera_transport_pub_ = image_transport::create_camera_publisher(node_.get(), topic_name_, video_qos_profile);


    //info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(topic_name_, 1);
    info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(info_topic_name_, rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos));

    disp_pub_ = node_->create_publisher<stereo_msgs::msg::DisparityImage>(topic_name_, rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos));
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
* Go_Disparity::openPub_noInfo()
----------------*/
void Go_Disparity::openPub_noInfo(std::shared_ptr<rclcpp::Node> node, std::string frame_name, std::string topic_name, int qos){

    node_=node;
    frame_name_ = frame_name;
    topic_name_ = topic_name;

    Que_Recv::pub_ready_=true;

    noInfo_f_=true;

}

//-----
// these code's base is from depthai-ros/depthai_bridge/src/DisparityConverter.cpp
//  DisparityConverter::toRosMsg()
//-----
/*--------------
* Go_Disparity::feedImages()
----------------*/
void Go_Disparity::feedImages(std::shared_ptr<dai::ADatatype> &Data){
    if(debug_f_){
        std::cout << "feedImagesPub() name:=" << Que_Recv::name_ << std::endl;
        std::cout << " frame_name_:"<< frame_name_ << std::endl;
    }
    auto inData = std::dynamic_pointer_cast<dai::ImgFrame>(Data);

    rclcpp::Time capture_time = node_->now();
    cv::Mat mat, output;

    //std::cout << " inData->getType()=" << inData->getType() << std::endl;
    //printf("%s",inData->getType());

    //ImageMsgs::Image outImageMsg;
    //StdMsgs::Header header;
    //header.frame_id = _frameName;

    cv::Size size = {0, 0};
    int type = 0;

    //std::shared_ptr<sensor_msgs::msg::Image> image = std::make_shared<sensor_msgs::msg::Image>();

    //DisparityMsgs::DisparityImage outDispImageMsg;
    std::shared_ptr<stereo_msgs::msg::DisparityImage> image = std::make_shared<stereo_msgs::msg::DisparityImage>();
    image->header.stamp = capture_time;
    image->header.frame_id = frame_name_;
    image->f = focalLength_;
    image->min_disparity = focalLength_ * baseline_ / maxDepth_;
    image->max_disparity = focalLength_ * baseline_ / minDepth_;

    image->t = baseline_ / 100.0;  // converting cm to meters

    //ImageMsgs::Image& outImageMsg = outDispImageMsg.image;
    //outDispImageMsg.header.stamp = getFrameTime(_rosBaseTime, _steadyBaseTime, tstamp);
    //image->header.stamp = getFrameTime(rosBaseTime_, steadyBaseTime_, tstamp);

    //outImageMsg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    image->image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;

    //outImageMsg.header = outDispImageMsg.header;
    image->image.header = image->header;

    if(inData->getType() == dai::RawImgFrame::Type::RAW8) {
        if(debug_f_){
            std::cout<< " inData->getType()==dai::RawImgFrame::Type::RAW8" << std::endl;
        }

        //outDispImageMsg.delta_d = 1.0;
        image->delta_d = 1.0;
        size_t size = inData->getData().size() * sizeof(float);
        //outImageMsg.data.resize(size);
        image->image.data.resize(size);
        //outImageMsg.height = inData->getHeight();
        image->image.height = inData->getHeight();
        //outImageMsg.width = inData->getWidth();
        image->image.width = inData->getWidth();
        //outImageMsg.step = size / inData->getHeight();
        image->image.step = size / inData->getHeight();
        //outImageMsg.is_bigendian = true;
        image->image.is_bigendian = true;

        std::vector<float> convertedData(inData->getData().begin(), inData->getData().end());
        //unsigned char* imageMsgDataPtr = reinterpret_cast<unsigned char*>(outImageMsg.data.data());
        unsigned char* imageMsgDataPtr = reinterpret_cast<unsigned char*>(image->image.data.data());

        unsigned char* daiImgData = reinterpret_cast<unsigned char*>(convertedData.data());

        // TODO(Sachin): Try using assign since it is a vector
        // img->data.assign(packet.data->cbegin(), packet.data->cend());
        memcpy(imageMsgDataPtr, daiImgData, size);

    } 
    else {
        //outDispImageMsg.delta_d = 1.0 / 32.0;
        image->delta_d = 1.0 / 32.0;

        size_t size = inData->getHeight() * inData->getWidth() * sizeof(float);
        //outImageMsg.data.resize(size);
        image->image.data.resize(size);

        //outImageMsg.height = inData->getHeight();
        image->image.height = inData->getHeight();

        //outImageMsg.width = inData->getWidth();
        image->image.width = inData->getWidth();

        //outImageMsg.step = size / inData->getHeight();
        image->image.step = size / inData->getHeight();

        //outImageMsg.is_bigendian = true;
        image->image.is_bigendian = true;

        unsigned char* daiImgData = reinterpret_cast<unsigned char*>(inData->getData().data());

        std::vector<int16_t> raw16Data(inData->getHeight() * inData->getWidth());
        unsigned char* raw16DataPtr = reinterpret_cast<unsigned char*>(raw16Data.data());
        memcpy(raw16DataPtr, daiImgData, inData->getData().size());
        std::vector<float> convertedData;
        std::transform(
            raw16Data.begin(), raw16Data.end(), std::back_inserter(convertedData), [](int16_t disp) -> std::size_t { return static_cast<float>(disp) / 32.0; });

        //unsigned char* imageMsgDataPtr = reinterpret_cast<unsigned char*>(outImageMsg.data.data());
        unsigned char* imageMsgDataPtr = reinterpret_cast<unsigned char*>(image->image.data.data());

        unsigned char* convertedDataPtr = reinterpret_cast<unsigned char*>(convertedData.data());
        memcpy(imageMsgDataPtr, convertedDataPtr, size);
    }
    //outDispImageMsgs.push_back(outDispImageMsg);

    if(trace_){
        cnt_++;
        rclcpp::Duration elapsed = capture_time - start_t_;
        if(elapsed.seconds() >= sec_dur_){
            std::cout << " oak-d_camera: "<< Que_Recv::name_ << " rate: " << cnt_/(int)sec_dur_ << std::endl;
            start_t_ = capture_time;
            cnt_=0;
        }
    }

    if(debug_f_){
        std::cout << " frame_name_:"<< frame_name_ << std::endl;
        std::cout << " image->header.frame_id:"<< image->header.frame_id << std::endl;
    }
    if(noInfo_f_!=true)
        sendInfo(capture_time, image);

}


/*--------------
* Go_Disparity::sendInfo() for DisparityImage

https://github.com/roboception/rc_genicam_driver_ros2/tree/master
https://github.com/roboception/rc_genicam_driver_ros2/blob/master/src/publishers/disparity_publisher.cpp
----------------*/
void Go_Disparity::sendInfo(rclcpp::Time time, std::shared_ptr<stereo_msgs::msg::DisparityImage> const & img){
    if(debug_f_)
        std::cout << " called Go_Publish::sendInfo()" << std::endl;

    //std::shared_ptr<sensor_msgs::msg::CameraInfo> info = std::make_shared<sensor_msgs::msg::CameraInfo>(info_mgr_->getCameraInfo());
    //printf("%s",info_left);

    if (!checkCameraInfo(*img, *info_)) {
        *info_ = sensor_msgs::msg::CameraInfo{};
        info_->height = img->image.height;
        info_->width = img->image.width;
    }

    info_->header.frame_id = frame_name_;
    info_->header.stamp = time;
    // add by nishi 2024.5.4
    img->header.stamp = time;
    img->header.frame_id=frame_name_;

    //trace_sts_=20;

    info_pub_->publish(*info_);
    disp_pub_->publish(*img);

    //if(intra_){
    //    left_pub_->publish(*img_l);
    //    left_info_pub_->publish(*info_left);
    //}
    //else{
        // http://docs.ros.org/en/jade/api/image_transport/html/classimage__transport_1_1CameraPublisher.html
        // https://github.com/ros-perception/image_transport_tutorials
        //camera_transport_pub_.publish(img, info);

    //trace_sts_=23;
    //}

}

}