/*
* stereo_camera.cpp
*
* depthai_ros_my/src/stereo_camera_test2.cpp
*
* original from
* depthai-core/examples/StereoDepth/stereo_depth_video.cpp
* http://docs.ros.org/en/noetic/api/depthai/html/opencv_2ImgFrame_8cpp_source.html
*/

#include <cstdio>
#include <functional>
#include <iostream>
#include <tuple>

#include "camera_info_manager/camera_info_manager.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"

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

#include "depthai_ros_my/camera_com.hpp"

#define USE_CAMER_CONTROL


std::tuple<dai::Pipeline, int, int, float> createPipeline(
    bool withDepth, bool lrcheck, bool extended, bool subpixel, int confidence, int LRchecktresh, std::string resolution,int rate,bool outputDepth,bool outputRectified) {

    // Create pipeline
    dai::Pipeline pipeline;
    dai::node::MonoCamera::Properties::SensorResolution monoResolution;

    //bool outputDepth=false;
    //bool outputRectified=true;

    //std::cout << " createPipeline()" << std::endl;

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = withDepth ? pipeline.create<dai::node::StereoDepth>() : nullptr;

    auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = pipeline.create<dai::node::XLinkOut>();

    auto xoutDisp = withDepth ? pipeline.create<dai::node::XLinkOut>(): nullptr;
    auto xoutDepth = withDepth ? pipeline.create<dai::node::XLinkOut>(): nullptr;
    auto xoutRectifL = withDepth ? pipeline.create<dai::node::XLinkOut>(): nullptr;
    auto xoutRectifR = withDepth ? pipeline.create<dai::node::XLinkOut>(): nullptr;

    #if defined(USE_CAMER_CONTROL)
        auto controlIn = pipeline.create<dai::node::XLinkIn>();
        controlIn->setStreamName("control");
        // Linking
        controlIn->out.link(monoRight->inputControl);
        controlIn->out.link(monoLeft->inputControl);
    #endif

    // XLinkOut
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");

    if(withDepth) {
        xoutDisp->setStreamName("disparity");
        xoutDepth->setStreamName("depth");
        xoutRectifL->setStreamName("rectified_left");
        xoutRectifR->setStreamName("rectified_right");
    }


    //std::cout << " createPipeline() :#5 " << std::endl;

    int width, height;
    if(resolution == "720p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P;
        width = 1280;
        height = 720;
    } else if(resolution == "400p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;
        width = 640;
        height = 400;
    } else if(resolution == "800p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P;
        width = 1280;
        height = 800;
    } else if(resolution == "480p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P;
        width = 640;
        height = 480;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid parameter. -> monoResolution: %s", resolution.c_str());
        throw std::runtime_error("Invalid mono camera resolution.");
    }

    // Properties
    monoLeft->setResolution(monoResolution);
    monoLeft->setCamera("left");
    monoRight->setResolution(monoResolution);
    monoRight->setCamera("right");

    // add by nishi 2024.5.5
    monoLeft->setFps(rate);
    monoRight->setFps(rate);


    if(withDepth) {
        // StereoDepth
        stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
        stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
        // stereo->setInputResolution(1280, 720);
        stereo->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_5x5);
        stereo->setLeftRightCheck(lrcheck);
        stereo->setExtendedDisparity(extended);
        stereo->setSubpixel(subpixel);

        // Linking CAM -> STEREO
        monoLeft->out.link(stereo->left);
        monoRight->out.link(stereo->right);

        stereo->syncedLeft.link(xoutLeft->input);
        stereo->syncedRight.link(xoutRight->input);

        stereo->disparity.link(xoutDisp->input);

        if(outputRectified) {
            stereo->rectifiedLeft.link(xoutRectifL->input);
            stereo->rectifiedRight.link(xoutRectifR->input);
        }
        if(outputDepth) {
            stereo->depth.link(xoutDepth->input);
        }
    } 
    else {
        // Link plugins CAM -> XLINK
        monoLeft->out.link(xoutLeft->input);
        monoRight->out.link(xoutRight->input);
    }
    // Disparity range is used for normalization
    float disparityMultiplier = withDepth ? 255 / stereo->initialConfig.getMaxDisparity() : 0;

    //std::cout << " createPipeline() :#99" << std::endl;
    return std::make_tuple(pipeline, width, height, disparityMultiplier);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("stereo_camera_test2");

    std::cout << "start stereo_camera_test2" << std::endl;

    bool outputDepth=false;
    bool outputRectified=true;


    std::string tfPrefix, mode, monoResolution;
    bool lrcheck, extended, subpixel, enableDepth;
    int confidence, LRchecktresh;
    int monoWidth, monoHeight;
    float disparityMultiplier;
    dai::Pipeline pipeline;

    int rate, queue_size;   // add by  nishi 2024.5..6
    int qos;
    bool trace;

    bool auto_exp;
    // Defaults and limits for manual focus/exposure controls
    int expTime = 20000;
    int expMin = 1;
    int expMax = 33000;

    int sensIso = 800;
    int sensMin = 100;
    int sensMax = 1600;

    node->declare_parameter("qos", 1);    // add by nishi 2024.5.6
    node->declare_parameter("tf_prefix", "oak");
    node->declare_parameter("mode", "depth");
    node->declare_parameter("lrcheck", true);
    node->declare_parameter("extended", false);
    node->declare_parameter("subpixel", true);
    node->declare_parameter("confidence", 200);
    node->declare_parameter("LRchecktresh", 5);
    node->declare_parameter("monoResolution", "720p");

    node->declare_parameter("rate", 30);    // add by nishi 2024.5.6
    node->declare_parameter("queue_size", 30);    // add by nishi 2024.5.6
    node->declare_parameter("trace", false);

    node->declare_parameter("auto_exp", false);
    node->declare_parameter("expTime", 20000);    // add by nishi 2024.5.18
    node->declare_parameter("sensIso", 800);    // add by nishi 2024.5.18

    node->get_parameter("qos", qos);      // add by nishi 2024.5.5
    node->get_parameter("tf_prefix", tfPrefix);
    node->get_parameter("mode", mode);
    node->get_parameter("lrcheck", lrcheck);
    node->get_parameter("extended", extended);
    node->get_parameter("subpixel", subpixel);
    node->get_parameter("confidence", confidence);
    node->get_parameter("LRchecktresh", LRchecktresh);
    node->get_parameter("monoResolution", monoResolution);

    node->get_parameter("rate", rate);      // add by nishi 2024.5.5
    node->get_parameter("queue_size", queue_size);      // add by nishi 2024.5.5
    node->get_parameter("trace", trace);

    node->get_parameter("auto_exp", auto_exp);
    node->get_parameter("expTime", expTime);      // add by nishi 2024.5.18
    node->get_parameter("sensIso", sensIso);      // add by nishi 2024.5.18

    std::cout << " queue_size:"<< queue_size << std::endl;

    if(mode == "depth") {
        enableDepth = true;
    } 
    else {
        enableDepth = false;
    }

    // test by nishi 2024.5.13
    enableDepth = false;

    //-------------------
    // set up oak-d camera device
    //-------------------
    std::tie(pipeline, monoWidth, monoHeight, disparityMultiplier) = createPipeline(enableDepth, lrcheck, extended, subpixel, confidence, LRchecktresh, monoResolution, rate,outputDepth,outputRectified);

    dai::Device device(pipeline);

    auto leftQueue = device.getOutputQueue("left", queue_size, false);
    auto rightQueue = device.getOutputQueue("right", queue_size, false);

    auto dispQueue = enableDepth ? device.getOutputQueue("disparity", queue_size, false) : nullptr;
    auto depthQueue = enableDepth ? device.getOutputQueue("depth", queue_size, false) : nullptr;

    auto rectifLeftQueue = enableDepth ? device.getOutputQueue("rectified_left", queue_size, false) : nullptr;
    auto rectifRightQueue = enableDepth ? device.getOutputQueue("rectified_right", queue_size, false) : nullptr;

    // Disparity range is used for normalization
    //float disparityMultiplier = enableDepth ? 255 / stereo->initialConfig.getMaxDisparity() : 0;

    #if defined(USE_CAMER_CONTROL)
        //auto controlQueue = device.getInputQueue(controlIn->getStreamName());
        auto controlQueue = device.getInputQueue("control");
    #endif

    #if defined(USE_CAMER_CONTROL)
        //------------------
        // set up camera
        // /home/nishi/local/git-download/depthai-core/examples/MonoCamera/mono_camera_control.cpp
        // iso 800  ->  sensIso
        // exposure, time: 20000  -> expTime
        dai::CameraControl ctrl;
        // AutoExposure
        if(auto_exp){
            std::cout << " auto_exp: true"<< std::endl;
            ctrl.setAutoExposureEnable();
            controlQueue->send(ctrl);
        }
        else{
            std::cout << " expTime: "<< expTime <<" sensIso: "<< sensIso << std::endl;
            ctrl.setManualExposure(expTime, sensIso);
            controlQueue->send(ctrl);
        }
    #endif

    //------------------
    // set up camera info
    //------------------
    auto calibrationHandler = device.readCalibration();

    //#define USE_ORG_CONVERTER
    #if defined(USE_ORG_CONVERTER)
        dai::rosBridge::ImageConverter converter(tfPrefix + "_left_camera_optical_frame", true);
        sensor_msgs::msg::CameraInfo leftCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_B, monoWidth, monoHeight);
        dai::rosBridge::ImageConverter rightconverter(tfPrefix + "_right_camera_optical_frame", true);
        auto rightCameraInfo = rightconverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, monoWidth, monoHeight);
    #else
        camera_com::CameraTools camera_tools;
        //camera_tools._reverseStereoSocketOrder = true;   // rtabmap_ros の rect_img のときは必要。
        auto leftCameraInfo = camera_tools.calibrationToCameraInfo_my(calibrationHandler, dai::CameraBoardSocket::CAM_B, monoWidth, monoHeight);
        auto rightCameraInfo = camera_tools.calibrationToCameraInfo_my(calibrationHandler, dai::CameraBoardSocket::CAM_C, monoWidth, monoHeight);
    #endif

    //std::cout << "Start while()" << std::endl;

    bool im_ok=false;

    //stereo_com::Que_Recv que_recv_left,que_recv_right;
    camera_com::Go_Publish go_pub_left,go_pub_right;
    go_pub_left.init(leftQueue);
    go_pub_left.openPub(node, tfPrefix + "_left_camera_optical_frame", "left/image_raw", qos, leftCameraInfo,trace);
    go_pub_right.init(rightQueue);
    go_pub_right.openPub(node, tfPrefix + "_right_camera_optical_frame", "right/image_raw", qos, rightCameraInfo);

    camera_com::Que_Recv dispQueue_recv, depthQueue_recv,
            rectifLeftQueue_recv,rectifRightQueue_recv;

    if(enableDepth){
        dispQueue_recv.init(dispQueue);
        if(outputDepth){
            depthQueue_recv.init(depthQueue);
        }
        if(outputRectified) {
            rectifLeftQueue_recv.init(rectifLeftQueue);
            rectifRightQueue_recv.init(rectifRightQueue);
        }
    }

    rclcpp::spin(node);
    return 0;
}