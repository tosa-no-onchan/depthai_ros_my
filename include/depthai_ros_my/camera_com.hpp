/*
* depthar-ros_my/camera_com.hpp
*/

#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"


#include <cstdio>
#include <iostream>
#include <tuple>

//#include <sensor_msgs/Image.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"

// add by nishi 2024.6.16 for disparity
#include <stereo_msgs/msg/disparity_image.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include "camera_info_manager/camera_info_manager.hpp"

#include "image_transport/image_transport.hpp"


// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

#include <opencv2/opencv.hpp>

#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/DisparityConverter.hpp"
#include "depthai_bridge/ImageConverter.hpp"

#include "depthai-shared/datatype/RawImgFrame.hpp"

//#include <vision_msgs/msg/detection2_d_array.hpp>
#include "vision_msgs/msg/detection2_d_array.hpp"

#include "utility/utility.hpp"


namespace camera_com {

class CameraTools{
public:

    //------
    // calibrationToCameraInfo_my
    // orginal
    //  src/depthai-ros/depthai_bridge/src/ImageConverter.cpp
    //    ImageConverter::calibrationToCameraInfo()
    //------
    sensor_msgs::msg::CameraInfo calibrationToCameraInfo_my(dai::CalibrationHandler calibHandler,
                                                dai::CameraBoardSocket cameraId,
                                                int width = -1,
                                                int height = -1,
                                                dai::Point2f topLeftPixelId = dai::Point2f(),
                                                dai::Point2f bottomRightPixelId = dai::Point2f());

    // for calibrationToCameraInfo_my
    bool _alphaScalingEnabled = false;        // image_rect のときは、どちらでもOK  どちらも大差ない。
    //bool _alphaScalingEnabled = true;           // image_rect のときは、どちらでもOK  (こちらが、depth& pointcloud と一致するのか?)

    bool _reverseStereoSocketOrder = false;
    //bool _reverseStereoSocketOrder = true;    // rtab_map_ros image_rect のときは、こちらを使う事。 そうしないと、 rtabmap_ros でエラーになる。

    double _alphaScalingFactor = 0.0;

private:

};

/*
* class Que_Recv
*/
class Que_Recv{
public:
    Que_Recv(){}
    //void init(std::shared_ptr<dai::DataOutputQueue> output_queue,std::string name, bool is_debug=false){
    void init(std::shared_ptr<dai::DataOutputQueue> output_queue, bool is_debug=false){
        output_queue_=output_queue;
        //name_=name;
        is_debug_=is_debug;
        name_= output_queue_->getName();
        set_callback();
    };

    void set_callback(){
        output_queue_->addCallback(std::bind(&Que_Recv::dataCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

    //-----------
    // 1) image
    // reffer /home/nishi/colcon_ws/src/depthai-ros/depthai_bridge/include/depthai_bridge/BridgePublisher.hpp
    //  void BridgePublisher<RosMsg, SimMsg>::daiCallback(std::string name, std::shared_ptr<ADatatype> data)
    // 2) detection
    // reffer /home/nishi/colcon_ws/src/depthai-ros/depthai_ros_driver/include/depthai_ros_driver/dai_nodes/nn/detection.hpp
    //  void detectionCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data)
    //-----------
    void dataCallback(std::string name, std::shared_ptr<dai::ADatatype> data){
        //auto daiDataPtr = std::dynamic_pointer_cast<dai::ImgFrame>(data);
        //auto inDet = std::dynamic_pointer_cast<dai::ImgDetections>(data);
        if(is_debug_){
            std::cout << "dataCallback() name:=" << name << std::endl;
        }
        if(pub_ready_){
            //feedImages(name,daiDataPtr);
            //feedImages(daiDataPtr);
            feedImages(data);
        }
    }

    //virtual void feedImages(std::shared_ptr<dai::ImgFrame> &inData){}
    virtual void feedImages(std::shared_ptr<dai::ADatatype> &Data){}

    //std::string name_;
    std::shared_ptr<dai::DataOutputQueue> output_queue_;
    bool is_debug_;
    bool pub_ready_=false;

    std::string name_;

private:
    //std::shared_ptr<rclcpp::Node> node_;
    //rclcpp::TimerBase::SharedPtr timer_;
    //rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr publisher_uint32_;
    //u_int32_t no_;
};

/*----------------------
* class Go_Publish
-----------------------*/
class Go_Publish: public Que_Recv{
public:

    std::shared_ptr<rclcpp::Node> node_;
    bool rgb2grey_=false;
    Go_Publish(){}

    //void init(std::shared_ptr<dai::DataOutputQueue> output_queue, bool is_debug=false){
    //    Que_Recv::init(output_queue, is_debug);
    //};

    void openPub(std::shared_ptr<rclcpp::Node> node, std::string frame_name, std::string topic_name, int qos, sensor_msgs::msg::CameraInfo &cameraInfo,bool trace=false);
    void openPub_noInfo(std::shared_ptr<rclcpp::Node> node, std::string frame_name, std::string topic_name, int qos);
    void sendInfo(rclcpp::Time time, std::shared_ptr<sensor_msgs::msg::Image> const & img);

    //void feedImages(std::shared_ptr<dai::ImgFrame> &inData);
    void feedImages(std::shared_ptr<dai::ADatatype> &Data);

    void set_debug(){
        debug_f_=true;
    }

    bool checkCameraInfo(sensor_msgs::msg::Image const & img, sensor_msgs::msg::CameraInfo const & ci)
    {
        return ci.width == img.width && ci.height == img.height;
    }

    // add by nishi 2024.6.22
    void set_resize(bool is_resize=true){
        is_resize_ = is_resize;
    }

    //sensor_msgs::msg::CameraInfo calibrationToCameraInfo(dai::CalibrationHandler calibHandler, dai::CameraBoardSocket cameraId, int width, int height, Point2f topLeftPixelId, Point2f bottomRightPixelId);


  private:
    bool debug_f_=false;
    bool noInfo_f_= false;
    bool trace_ = false;
    rclcpp::Time start_t_;
    double sec_dur_ = 3.0;
    int cnt_=0;

    // add by nishi 22024.6.22
    bool is_resize_ = false;
    uint32_t height_,width_;


    std::string frame_name_;
    std::string topic_name_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> info_mgr_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    // https://github.com/ros-perception/image_common
    // https://github.com/ros-perception/image_common/blob/rolling/image_transport/src/camera_publisher.cpp
    image_transport::CameraPublisher camera_transport_pub_;

};


/*----------------------
* class Go_DtectPublish
-----------------------*/
class Go_DtectPublish: public Que_Recv{
public:

    std::shared_ptr<rclcpp::Node> node_;
    bool rgb2grey_=false;
    Go_DtectPublish(){}

    //void init(std::shared_ptr<dai::DataOutputQueue> output_queue, bool is_debug=false){
    //    Que_Recv::init(output_queue, is_debug);
    //};

    void openPub(std::shared_ptr<rclcpp::Node> node, std::string frame_name, std::string topic_name, int qos, sensor_msgs::msg::CameraInfo &cameraInfo,bool trace=false);
    void openPub_noInfo(std::shared_ptr<rclcpp::Node> node, std::string frame_name, std::string topic_name, int qos,int width, int height , bool normalized=false);
    void sendInfo(rclcpp::Time time, std::shared_ptr<sensor_msgs::msg::Image> const & img);
    //void feedImages(std::shared_ptr<dai::ImgFrame> &inData);
    void feedImages(std::shared_ptr<dai::ADatatype> &Data);

    void set_debug(){
        debug_f_=true;
    }

    void set_labelmap(std::vector<std::string> *labelMap){
        labelMap_=labelMap;
        is_labelMap_=true;
    }

    bool checkCameraInfo(sensor_msgs::msg::Image const & img, sensor_msgs::msg::CameraInfo const & ci)
    {
        return ci.width == img.width && ci.height == img.height;
    }

    //sensor_msgs::msg::CameraInfo calibrationToCameraInfo(dai::CalibrationHandler calibHandler, dai::CameraBoardSocket cameraId, int width, int height, Point2f topLeftPixelId, Point2f bottomRightPixelId);


  private:
    bool debug_f_=false;
    bool noInfo_f_= false;
    bool trace_ = false;
    rclcpp::Time start_t_;
    double sec_dur_ = 3.0;
    int cnt_=0;

    int width_, height_;
    bool normalized_;

    bool is_labelMap_=false;
    std::vector<std::string> *labelMap_;

    std::string frame_name_;
    std::string topic_name_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> info_mgr_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    image_transport::CameraPublisher camera_transport_pub_;

    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detect_pub_;

};


/*----------------------
* class Go_Disparity
-----------------------*/
class Go_Disparity: public Que_Recv{
public:

    std::shared_ptr<rclcpp::Node> node_;
    bool rgb2grey_=false;
    Go_Disparity(float focalLength, float baseline = 7.5, float minDepth = 80, float maxDepth = 1100, bool getBaseDeviceTimestamp = false){
        focalLength_ = focalLength;
        baseline_ = (baseline / 100.0);
        minDepth_ = (minDepth / 100.0);
        maxDepth_ = (maxDepth / 100.0);
        steadyBaseTime_ = std::chrono::steady_clock::now();
        getBaseDeviceTimestamp_ = getBaseDeviceTimestamp;
        rosBaseTime_ = rclcpp::Clock().now();
    }

    //void init(std::shared_ptr<dai::DataOutputQueue> output_queue, bool is_debug=false){
    //    Que_Recv::init(output_queue, is_debug);
    //};

    void openPub(std::shared_ptr<rclcpp::Node> node, std::string frame_name, std::string topic_name,std::string info_topic_name, int qos, sensor_msgs::msg::CameraInfo &cameraInfo,bool trace=false);
    void openPub_noInfo(std::shared_ptr<rclcpp::Node> node, std::string frame_name, std::string topic_name, int qos);
    //void sendInfo(rclcpp::Time time, std::shared_ptr<sensor_msgs::msg::Image> const & img);
    void sendInfo(rclcpp::Time time, std::shared_ptr<stereo_msgs::msg::DisparityImage> const & img);

    //void feedImages(std::shared_ptr<dai::ImgFrame> &inData);
    void feedImages(std::shared_ptr<dai::ADatatype> &Data);

    void set_debug(){
        debug_f_=true;
    }

    void set_labelmap(std::vector<std::string> *labelMap){
        labelMap_=labelMap;
        is_labelMap_=true;
    }


    bool checkCameraInfo(stereo_msgs::msg::DisparityImage const & img, sensor_msgs::msg::CameraInfo const & ci)
    {
        return ci.width == img.image.width && ci.height == img.image.height;
    }

    //sensor_msgs::msg::CameraInfo calibrationToCameraInfo(dai::CalibrationHandler calibHandler, dai::CameraBoardSocket cameraId, int width, int height, Point2f topLeftPixelId, Point2f bottomRightPixelId);


  private:
    bool debug_f_=false;
    bool noInfo_f_= false;
    bool trace_ = false;
    rclcpp::Time start_t_;
    double sec_dur_ = 3.0;
    int cnt_=0;

    int width_, height_;
    bool normalized_;

    bool is_labelMap_=false;
    std::vector<std::string> *labelMap_;

    std::string frame_name_;
    float focalLength_ = 882.2;
    float baseline_ = 7.5;
    float minDepth_ = 80;
    float maxDepth_;
    std::chrono::time_point<std::chrono::steady_clock> steadyBaseTime_;
    rclcpp::Time rosBaseTime_;
    bool getBaseDeviceTimestamp_;


    std::string topic_name_;
    std::string info_topic_name_;
    //sensor_msgs::msg::CameraInfo cameraInfo_;

    //std::shared_ptr<sensor_msgs::msg::CameraInfo> info = std::make_shared<sensor_msgs::msg::CameraInfo>(info_mgr_->getCameraInfo());
    std::shared_ptr<sensor_msgs::msg::CameraInfo> info_;

    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    //std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> pub;


    //rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detect_pub_;
    std::shared_ptr<rclcpp::Publisher<stereo_msgs::msg::DisparityImage>> disp_pub_;

};

}