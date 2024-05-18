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
//#include <sensor_msgs/CameraInfo.hpp>
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

    void dataCallback(std::string name, std::shared_ptr<dai::ADatatype> data){
        auto daiDataPtr = std::dynamic_pointer_cast<dai::ImgFrame>(data);
        if(is_debug_){
            std::cout << "dataCallback() name:=" << name << std::endl;
        }
        if(pub_ready_){
            //feedImages(name,daiDataPtr);
            feedImages(daiDataPtr);
        }
    }

    virtual void feedImages(std::shared_ptr<dai::ImgFrame> &inData){}

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
    void feedImages(std::shared_ptr<dai::ImgFrame> &inData);

    void set_debug(){
        debug_f_=true;
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


    std::string frame_name_;
    std::string topic_name_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> info_mgr_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    image_transport::CameraPublisher camera_transport_pub_;

};


}