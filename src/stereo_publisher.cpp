/*
* depthai_ros_my/stereo_publisher.cpp
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

// Inludes common necessary includes for development using depthai library
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/DisparityConverter.hpp"
#include "depthai_bridge/ImageConverter.hpp"

std::tuple<dai::Pipeline, int, int> createPipeline(
    bool withDepth, bool lrcheck, bool extended, bool subpixel, int confidence, int LRchecktresh, std::string resolution,int rate) {
    dai::Pipeline pipeline;
    dai::node::MonoCamera::Properties::SensorResolution monoResolution;
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = pipeline.create<dai::node::XLinkOut>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();

    // XLinkOut
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");

    if(withDepth) {
        xoutDepth->setStreamName("depth");
    } else {
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
    auto node = rclcpp::Node::make_shared("stereo_node");

    std::string tfPrefix, mode, monoResolution;
    bool lrcheck, extended, subpixel, enableDepth;
    int confidence, LRchecktresh;
    int monoWidth, monoHeight;
    dai::Pipeline pipeline;

    int rate, queue_size;   // add by  nishi 2024.5..6

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

    if(mode == "depth") {
        enableDepth = true;
    } 
    else {
        enableDepth = false;
    }

    std::tie(pipeline, monoWidth, monoHeight) = createPipeline(enableDepth, lrcheck, extended, subpixel, confidence, LRchecktresh, monoResolution, rate);
    dai::Device device(pipeline);
    
    auto leftQueue = device.getOutputQueue("left", queue_size, false);
    auto rightQueue = device.getOutputQueue("right", queue_size, false);
    std::shared_ptr<dai::DataOutputQueue> stereoQueue;
    if(enableDepth) {
        stereoQueue = device.getOutputQueue("depth", queue_size, false);
    } 
    else {
        stereoQueue = device.getOutputQueue("disparity", queue_size, false);
    }

    auto calibrationHandler = device.readCalibration();

    auto boardName = calibrationHandler.getEepromData().boardName;
    if(monoHeight > 480 && boardName == "OAK-D-LITE") {
        monoWidth = 640;
        monoHeight = 480;
    }

    dai::rosBridge::ImageConverter converter(tfPrefix + "_left_camera_optical_frame", true);
    converter.reverseStereoSocketOrder();       // rtabmap_ros rect_img のときは必要。 2024.5.15 by nishi
    auto leftCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_B, monoWidth, monoHeight);

    // test by nishi 2024.5.7
    //converter.convertFromBitstream(dai::RawImgFrame::Type::GRAY8);
    //converter.convertFromBitstream(dai::RawImgFrame::Type::RAW8);

    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> leftPublish(
        leftQueue,
        node,
        std::string("left/image_rect"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &converter, std::placeholders::_1, std::placeholders::_2),
        queue_size,
        leftCameraInfo,
        "left");

    leftPublish.addPublisherCallback();     //  std::shared_ptr<dai::DataOutputQueue> _daiMessageQueue->addCallback() で callback に登録か

    dai::rosBridge::ImageConverter rightconverter(tfPrefix + "_right_camera_optical_frame", true);
    rightconverter.reverseStereoSocketOrder();       // rtabmap_ros rect_img のときは必要。 2024.5.15 by nishi
    //auto rightCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, monoWidth, monoHeight);
    // changed by nishi 2024.5.8
    auto rightCameraInfo = rightconverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, monoWidth, monoHeight);

    // test by nishi 2024.5.7
    //rightconverter.convertFromBitstream(dai::RawImgFrame::Type::GRAY8);
    //rightconverter.convertFromBitstream(dai::RawImgFrame::Type::RAW8);

    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rightPublish(
        rightQueue,
        node,
        std::string("right/image_rect"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rightconverter, std::placeholders::_1, std::placeholders::_2),
        queue_size,
        rightCameraInfo,
        "right");

    rightPublish.addPublisherCallback();

    // add by nishi 2024.5.7
    //dai::rosBridge::ImageConverter depthconverter(tfPrefix + "_right_camera_optical_frame", true);

    if(mode == "depth") {
        dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> depthPublish(
            stereoQueue,
            node,
            std::string("stereo/depth"),
            std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                      &rightconverter,  // since the converter has the same frame name
                      //&depthconverter,  // since the converter has the same frame name
                                        // and image type is also same we can reuse it
                      std::placeholders::_1,
                      std::placeholders::_2),
            queue_size,
            rightCameraInfo,
            "stereo");
        depthPublish.addPublisherCallback();
        rclcpp::spin(node);
    } 
    else {
        dai::rosBridge::DisparityConverter dispConverter(tfPrefix + "_right_camera_optical_frame", 880, 7.5, 20, 2000);
        dai::rosBridge::BridgePublisher<stereo_msgs::msg::DisparityImage, dai::ImgFrame> dispPublish(
            stereoQueue,
            node,
            std::string("stereo/disparity"),
            std::bind(&dai::rosBridge::DisparityConverter::toRosMsg, &dispConverter, std::placeholders::_1, std::placeholders::_2),
            queue_size,
            rightCameraInfo,
            "stereo");
        dispPublish.addPublisherCallback();
        rclcpp::spin(node);
    }
    return 0;
}
