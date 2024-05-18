/*
* stereo_publisher_my.cpp
*
* depthai_ros_my/src/stereo_publisher_my.cpp
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

std::tuple<dai::Pipeline, int, int> createPipeline(
    bool withDepth, bool lrcheck, bool extended, bool subpixel, int confidence, int LRchecktresh, std::string resolution,int rate) {

    // Create pipeline
    dai::Pipeline pipeline;
    dai::node::MonoCamera::Properties::SensorResolution monoResolution;

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = pipeline.create<dai::node::XLinkOut>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();

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
        xoutDepth->setStreamName("depth");
    } 
    else {
        xoutDepth->setStreamName("disparity");
    }

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

    // MonoCamera
    monoLeft->setResolution(monoResolution);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    monoRight->setResolution(monoResolution);
    monoRight->setBoardSocket(dai::CameraBoardSocket::CAM_C);

    // add by nishi 2024.5.5
    monoLeft->setFps(rate);
    monoRight->setFps(rate);

    // StereoDepth
    stereo->initialConfig.setConfidenceThreshold(confidence);
    stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);

    // Link plugins CAM -> STEREO -> XLINK
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    stereo->rectifiedLeft.link(xoutLeft->input);
    stereo->rectifiedRight.link(xoutRight->input);

    if(withDepth) {
        stereo->depth.link(xoutDepth->input);
    } 
    else {
        stereo->disparity.link(xoutDepth->input);
    }
    return std::make_tuple(pipeline, width, height);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("stereo_publisher_my");

    std::cout << "stereo_publisher_my" << std::endl;

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

    //-------------------
    // set up oak-d camera device
    //-------------------
    std::tie(pipeline, monoWidth, monoHeight) = createPipeline(enableDepth, lrcheck, extended, subpixel, confidence, LRchecktresh, monoResolution, rate);

    dai::Device device(pipeline);

    auto leftQueue = device.getOutputQueue("left", queue_size, false);
    auto rightQueue = device.getOutputQueue("right", queue_size, false);

    std::string queue_name = enableDepth ? "depth": "disparity";
    auto stereoQueue = device.getOutputQueue(queue_name, queue_size, false);

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

    // if文の{}の中に置くと、うまく動きません。必ず直において下さい、
    camera_com::Go_Publish go_pub_left,go_pub_right;

    // if文の{}の中に置くと、うまく動きません。必ず直において下さい、
    camera_com::Go_Publish go_pub_depth,go_pub_disp;

    //------------------
    // set up camera info
    //------------------
    auto calibrationHandler = device.readCalibration();

    //#define USE_ORG_CONVERTER
    #if defined(USE_ORG_CONVERTER)
        dai::rosBridge::ImageConverter converter(tfPrefix + "_left_camera_optical_frame", true);
        converter.reverseStereoSocketOrder();       // rtabmap_ros rect_img のときは必要。 2024.5.15 by nishi
        sensor_msgs::msg::CameraInfo leftCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_B, monoWidth, monoHeight);
        dai::rosBridge::ImageConverter rightconverter(tfPrefix + "_right_camera_optical_frame", true);
        rightconverter.reverseStereoSocketOrder();    // rtabmap_ros rect_img のときは必要。 2024.5.15 by nishi
        auto rightCameraInfo = rightconverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, monoWidth, monoHeight);
    #else
        camera_com::CameraTools camera_tools;
        camera_tools._reverseStereoSocketOrder = true;   // rect_img のときは必要。
        sensor_msgs::msg::CameraInfo leftCameraInfo = camera_tools.calibrationToCameraInfo_my(calibrationHandler, dai::CameraBoardSocket::CAM_B, monoWidth, monoHeight);
        auto rightCameraInfo = camera_tools.calibrationToCameraInfo_my(calibrationHandler, dai::CameraBoardSocket::CAM_C, monoWidth, monoHeight);
    #endif
    //printf("%s",leftCameraInfo);

    //std::cout << "Start while()" << std::endl;

    //bool im_ok=false;

    //----------------
    // start camera publisher
    //----------------
    //camera_com::Que_Recv que_recv_left,que_recv_right;
    #define PUB_OK_X
    #if defined(PUB_OK_X)

        go_pub_left.init(leftQueue);
        //go_pub_left.set_debug();
        go_pub_left.openPub(node, tfPrefix + "_left_camera_optical_frame", "left/image_rect", qos, leftCameraInfo,trace);
        go_pub_right.init(rightQueue);
        go_pub_right.openPub(node, tfPrefix + "_right_camera_optical_frame", "right/image_rect", qos, rightCameraInfo);
    #else
        camera_com::Que_Recv que_recv_left,que_recv_right;
        que_recv_left.init(leftQueue);
        que_recv_right.init(rightQueue);
    #endif


    if(mode == "depth") {
        go_pub_depth.init(stereoQueue);
        //go_pub_depth.set_debug();
        go_pub_depth.openPub(node, tfPrefix + "_right_camera_optical_frame", "stereo/depth", qos, rightCameraInfo);
    }
    else{
        //camera_com::Que_Recv dispQueue_recv;
        //dispQueue_recv.init(stereoQueue);
        go_pub_disp.init(stereoQueue);
        //go_pub_disp.set_debug();
        go_pub_disp.openPub(node, tfPrefix + "_right_camera_optical_frame", "stereo/disparity", qos, rightCameraInfo);
    }
    rclcpp::spin(node);
    return 0;
}