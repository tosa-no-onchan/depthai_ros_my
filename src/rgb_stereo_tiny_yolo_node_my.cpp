/*
* rgb_stereo_tiny_yolo_node_my.cpp
*
* test program
* depthai_ros_my/src/rgb_stereo_tiny_yolo_node_my.cpp
*
* original from
* depthai-core/examples/StereoDepth/rgb_stereo_node.cpp
* http://docs.ros.org/en/noetic/api/depthai/html/opencv_2ImgFrame_8cpp_source.html
*
* 1. build on SBC and PC
*  $ colcon build --symlink-install --parallel-workers 1 --packages-select depthai_ros_my
*  $ . install/setup.bash
*
*/

#include <chrono>
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
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/DisparityConverter.hpp"
#include "depthai_bridge/ImageConverter.hpp"

// add by nishi
#include "depthai_ros_my/camera_com.hpp"

#include <fstream>
#include <nlohmann/json.hpp>

using namespace std::chrono_literals;

using json = nlohmann::json;


#define USE_CAMER_CONTROL

std::tuple<dai::Pipeline, int, int> createPipeline(bool lrcheck,
                                                   bool extended,
                                                   bool subpixel,
                                                   int confidence,
                                                   int LRchecktresh,
                                                   bool useVideo,
                                                   bool usePreview,
                                                   int previewWidth,
                                                   int previewHeight,
                                                   std::string mResolution,
                                                   std::string cResolution,
                                                   int rate,    // add by nishi 2024.5.5
                                                   std::string nnPath,   // add by nishi 2024.6.20
                                                   json &j_data,    // add b nishi 2024.7.1
                                                   bool syncNN
                                                   ) {
    dai::Pipeline pipeline;

    //-------
    // Set up Stereo Depth part.
    //-------
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    // create xlink connections
    // for depth output
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();

    // set StreamName for depth output
    xoutDepth->setStreamName("depth");

    int width, height;
    dai::node::MonoCamera::Properties::SensorResolution monoResolution;
    if(mResolution == "720p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P;
        width = 1280;
        height = 720;
    } else if(mResolution == "400p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;
        width = 640;
        height = 400;
    } else if(mResolution == "800p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P;
        width = 1280;
        height = 800;
    } else if(mResolution == "480p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P;
        width = 640;
        height = 480;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid parameter. -> monoResolution: %s", mResolution.c_str());
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

    // // Link plugins CAM -> STEREO -> XLINK
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);
    stereo->depth.link(xoutDepth->input);


    //-------
    // Set up Center Color Cam and Object detection part.
    //-------
    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    // add for yolo 2024.6.20
    //auto detectionNetwork = pipeline.create<dai::node::MobileNetDetectionNetwork>();
    auto detectionNetwork = pipeline.create<dai::node::YoloDetectionNetwork>();

    auto xlinkPreviewOut = pipeline.create<dai::node::XLinkOut>();
    auto xlinkVideoOut = pipeline.create<dai::node::XLinkOut>();

    auto nnOut = pipeline.create<dai::node::XLinkOut>();
    nnOut->setStreamName("detections");

    xlinkVideoOut->setStreamName("video");
    //xlinkVideoOut->input.setQueueSize(1);
    xlinkVideoOut->input.setQueueSize(2);

    xlinkPreviewOut->setStreamName("preview");

    #if defined(USE_CAMER_CONTROL)
        // add by nishi 2024.5.17
        auto controlIn = pipeline.create<dai::node::XLinkIn>();
        controlIn->setStreamName("control");
        // Linking
        // add by nishi 2024.5.17
        controlIn->out.link(colorCam->inputControl);
    #endif

    std::cout << " cResolution:" << cResolution << std::endl;


    dai::ColorCameraProperties::SensorResolution colorResolution;
    if(cResolution == "1080p") {
        colorResolution = dai::ColorCameraProperties::SensorResolution::THE_1080_P;
    }
    // add by nishi 2024.5.5
    else if(cResolution == "480p") {
        colorResolution = dai::ColorCameraProperties::SensorResolution::THE_1080_P;
        //colorResolution = dai::ColorCameraProperties::SensorResolution::THE_800_P;
    }
    else if(cResolution == "4K") {
        colorResolution = dai::ColorCameraProperties::SensorResolution::THE_4_K;
    }

    colorCam->setResolution(colorResolution);
    //colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);

    if(cResolution == "1080p") {
        colorCam->setVideoSize(1920, 1080);
    }
    // add by nishi 2024.5.5
    else if(cResolution == "480p") {
        // このサイズにすると、 yolov4 で検出ができなくなる。 1920x1080 が必要だ。
        //colorCam->setVideoSize(640, 480);
        colorCam->setVideoSize(1920, 1080);
    } 
    else {
        colorCam->setVideoSize(3840, 2160);
    }

    // 416x416
    colorCam->setPreviewSize(previewWidth, previewHeight);   // NN input
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    //colorCam->setFps(30);
    // changed by nishi 2024.5.5
    colorCam->setFps(rate);

    detectionNetwork->setConfidenceThreshold(0.5);

    int classes=80;
    if(j_data.is_null() == false){
        try{
            auto s = j_data["nn_config"]["NN_specific_metadata"]["classes"];
            //std::cout << "s:" << s << std::endl;
            classes=s;
        }
        catch(...){
            std::cout << "createPipeline() : #3 error" << std::endl;
        }
    }
    detectionNetwork->setNumClasses(classes);

    detectionNetwork->setCoordinateSize(4);

    if(j_data.is_null() == false){
        try{
            auto anchors = j_data["nn_config"]["NN_specific_metadata"]["anchors"];     // char *
            auto anchor_masks = j_data["nn_config"]["NN_specific_metadata"]["anchor_masks"];  // char *
            detectionNetwork->setAnchors(anchors);
            detectionNetwork->setAnchorMasks(anchor_masks);

            auto iou_threshold = j_data["nn_config"]["NN_specific_metadata"]["iou_threshold"];
            detectionNetwork->setIouThreshold(iou_threshold);
        }
        catch(...){
            std::cout << "createPipeline() : #4 error" << std::endl;
        }
    }
    else{
        detectionNetwork->setAnchors({10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319});
        detectionNetwork->setAnchorMasks({{"side26", {1, 2, 3}}, {"side13", {3, 4, 5}}});
        detectionNetwork->setIouThreshold(0.5f);
    }

    detectionNetwork->setBlobPath(nnPath);
    detectionNetwork->setNumInferenceThreads(2);
    detectionNetwork->input.setBlocking(false);

    if(useVideo) {
        colorCam->video.link(xlinkVideoOut->input);
    }

    // not use usePreview and mobilenet at same time? 
    if(usePreview) {
        colorCam->preview.link(xlinkPreviewOut->input);
    }
    else{
        // yolo detection
        // Properties
        colorCam->preview.link(detectionNetwork->input);
        if(syncNN)
            detectionNetwork->passthrough.link(xlinkPreviewOut->input);
        else
            colorCam->preview.link(xlinkPreviewOut->input);

        detectionNetwork->out.link(nnOut->input);
        //nn->outNetwork.link(nnNetworkOut->input);
    }

    return std::make_tuple(pipeline, width, height);
}

//std::vector<std::string> labelMap = {"background", "aeroplane", "bicycle",     "bird",  "boat",        "bottle", "bus",
//                                                  "car",        "cat",       "chair",       "cow",   "diningtable", "dog",    "horse",
//                                                  "motorbike",  "person",    "pottedplant", "sheep", "sofa",        "train",  "tvmonitor"};

std::vector<std::string> labelMap_80 = {
    "person",        "bicycle",      "car",           "motorbike",     "aeroplane",   "bus",         "train",       "truck",        "boat",
    "traffic light", "fire hydrant", "stop sign",     "parking meter", "bench",       "bird",        "cat",         "dog",          "horse",
    "sheep",         "cow",          "elephant",      "bear",          "zebra",       "giraffe",     "backpack",    "umbrella",     "handbag",
    "tie",           "suitcase",     "frisbee",       "skis",          "snowboard",   "sports ball", "kite",        "baseball bat", "baseball glove",
    "skateboard",    "surfboard",    "tennis racket", "bottle",        "wine glass",  "cup",         "fork",        "knife",        "spoon",
    "bowl",          "banana",       "apple",         "sandwich",      "orange",      "broccoli",    "carrot",      "hot dog",      "pizza",
    "donut",         "cake",         "chair",         "sofa",          "pottedplant", "bed",         "diningtable", "toilet",       "tvmonitor",
    "laptop",        "mouse",        "remote",        "keyboard",      "cell phone",  "microwave",   "oven",        "toaster",      "sink",
    "refrigerator",  "book",         "clock",         "vase",          "scissors",    "teddy bear",  "hair drier",  "toothbrush"};

// Manual exposure/focus set step
static constexpr int EXP_STEP = 500;  // us
static constexpr int ISO_STEP = 50;
static constexpr int LENS_STEP = 3;
static constexpr int WB_STEP = 200;


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("rgb_stereo_tiny_yolo_node_my");

    std::cout << "rgb_stereo_tiny_yolo_node_my" << std::endl;

    std::string tfPrefix, mode, monoResolution, colorResolution;
    bool lrcheck, extended, subpixel;
    bool useVideo, usePreview, useDepth;
    int confidence, LRchecktresh, previewWidth, previewHeight;
    float dotProjectormA, floodLightmA;

    int rate, queue_size;   // add by  nishi 2024.5..6
    int qos;
    bool trace,rgb2grey;


    bool auto_exp;
    int expTime = 20000;    // 27500
    int expMin = 1;
    int expMax = 33000;

    int sensIso = 800;      // 1000
    int sensMin = 100;
    int sensMax = 1600;

    int wbManual = 4000;
    int wbMin = 1000;
    int wbMax = 12000;

    std::string nnPath;
    bool normalized=true;

    std::string nnJson;
    json j_data;

    std::vector<std::string> labelMap;
    labelMap=labelMap_80;


    // if文の{}の中に置くと、うまく動きません。必ず直において下さい、
    camera_com::Go_Publish go_depthQueue_pub,go_previewQueue_pub, go_videoQueue_pub;

    camera_com::Go_DtectPublish go_dpub_nNetDataQueue;


    node->declare_parameter("qos", 1);    // add by nishi 2024.5.6

    node->declare_parameter("tf_prefix", "oak");
    node->declare_parameter("lrcheck", true);
    node->declare_parameter("extended", false);
    node->declare_parameter("subpixel", true);
    node->declare_parameter("confidence", 200);
    node->declare_parameter("LRchecktresh", 5);
    node->declare_parameter("monoResolution", "720p");
    node->declare_parameter("colorResolution", "1080p");
    node->declare_parameter("useVideo", true);
    node->declare_parameter("usePreview", false);
    node->declare_parameter("useDepth", true);
    node->declare_parameter("previewWidth", 416);
    node->declare_parameter("previewHeight", 416);
    node->declare_parameter("dotProjectormA", 0.0f);
    node->declare_parameter("floodLightmA", 0.0f);

    node->declare_parameter("rate", 30);    // add by nishi 2024.5.6
    node->declare_parameter("queue_size", 30);    // add by nishi 2024.5.6
    node->declare_parameter("trace", false);
    node->declare_parameter("rgb2grey", false);


    node->declare_parameter("auto_exp", false);
    node->declare_parameter("sensIso", 1000);
    node->declare_parameter("expTime", 27500);
    node->declare_parameter("nnPath", "");
    node->declare_parameter("normalized", normalized);
    node->declare_parameter<std::string>("nnJson", "");

    node->get_parameter("qos", qos);      // add by nishi 2024.5.5
    node->get_parameter("tf_prefix", tfPrefix);
    node->get_parameter("lrcheck", lrcheck);
    node->get_parameter("extended", extended);
    node->get_parameter("subpixel", subpixel);
    node->get_parameter("confidence", confidence);
    node->get_parameter("LRchecktresh", LRchecktresh);
    node->get_parameter("monoResolution", monoResolution);
    node->get_parameter("colorResolution", colorResolution);
    node->get_parameter("useVideo", useVideo);
    node->get_parameter("usePreview", usePreview);
    node->get_parameter("useDepth", useDepth);
    node->get_parameter("previewWidth", previewWidth);
    node->get_parameter("previewHeight", previewHeight);
    node->get_parameter("dotProjectormA", dotProjectormA);
    node->get_parameter("floodLightmA", floodLightmA);

    node->get_parameter("rate", rate);      // add by nishi 2024.5.5
    node->get_parameter("queue_size", queue_size);      // add by nishi 2024.5.5
    node->get_parameter("trace", trace);
    node->get_parameter("rgb2grey", rgb2grey);

    node->get_parameter("auto_exp", auto_exp);
    node->get_parameter("sensIso", sensIso);
    node->get_parameter("expTime", expTime);

    node->get_parameter("nnPath", nnPath);
    node->get_parameter("normalized", normalized);

    node->get_parameter("nnJson", nnJson);

    std::cout << " queue_size:"<< queue_size << std::endl;
    std::cout << " rate:"<< rate << std::endl;
    std::cout << "Using blob at path: " << nnPath << std::endl;

    int colorWidth, colorHeight;
    if(colorResolution == "1080p") {
        colorWidth = 1920;
        colorHeight = 1080;
    }
    // add by nishi 2024.5.5
    if(colorResolution == "480p") {
        colorWidth = 640;
        colorHeight = 480;
    }
    else if(colorResolution == "4K") {
        colorWidth = 3840;
        colorHeight = 2160;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid parameter. -> colorResolution: %s", colorResolution.c_str());
        throw std::runtime_error("Invalid color camera resolution.");
    }

    if(nnJson != ""){
        std::ifstream f(nnJson);
        try{
            j_data = json::parse(f);
            auto labels = j_data["mappings"]["labels"];
            labelMap = std::vector<std::string>(labels);
        }
        catch(...){
            std::cout << " nnJson access error"<< std::endl;
        }
    }

    dai::Pipeline pipeline;
    int monoWidth, monoHeight;

    //-------------------
    // set up oak-d camera device
    //-------------------
    std::tie(pipeline, monoWidth, monoHeight) = createPipeline(
        //lrcheck, extended, subpixel, confidence, LRchecktresh, useVideo, usePreview, previewWidth, previewHeight, monoResolution, colorResolution);
        // changed by nishi 2024.5.5
        lrcheck, extended, subpixel, confidence, LRchecktresh, useVideo, usePreview, previewWidth, previewHeight, monoResolution, colorResolution,rate, nnPath, j_data, true);
    dai::Device device(pipeline);

    //auto videoQueue = device.getOutputQueue("video", queue_size, false);    // changed by nishi 2024.5.6
    std::shared_ptr<dai::DataOutputQueue> videoQueue = device.getOutputQueue("video", queue_size, false);    // changed by nishi 2024.5.6

    auto depthQueue = device.getOutputQueue("depth", queue_size, false);   // changed by nishi 2024.5.6
    auto previewQueue = device.getOutputQueue("preview", queue_size, false);    // changed by nishi 2024.5.6

    std::shared_ptr<dai::DataOutputQueue> nNetDataQueue = device.getOutputQueue("detections", queue_size, false);
    //auto qNN = device.getOutputQueue("nnNetwork", queue_size, false);


    #if defined(USE_CAMER_CONTROL)
        // add by nishi 2024.5.17
        auto controlQueue = device.getInputQueue("control");
    #endif
    auto calibrationHandler = device.readCalibration();

    auto boardName = calibrationHandler.getEepromData().boardName;
    if(monoHeight > 480 && boardName == "OAK-D-LITE") {
        monoWidth = 640;
        monoHeight = 480;
    }

    rclcpp::TimerBase::SharedPtr timer;
    auto cb = [node, &device, &dotProjectormA, &floodLightmA]() {
        // rclcpp::Parameter p;
        float dotProjectormATemp, floodLightmATemp;
        node->get_parameter("dotProjectormA", dotProjectormATemp);
        node->get_parameter("floodLightmA", floodLightmATemp);
        if(dotProjectormATemp != dotProjectormA) {
            dotProjectormA = dotProjectormATemp;
            RCLCPP_INFO(node->get_logger(), "Updating Dot Projector current to %f", dotProjectormA);
            device.setIrLaserDotProjectorBrightness(static_cast<float>(dotProjectormA));
        }

        if(floodLightmATemp != floodLightmA) {
            floodLightmA = floodLightmATemp;
            RCLCPP_INFO(node->get_logger(), "Updating Flood Light current to %f", floodLightmA);
            device.setIrFloodLightBrightness(static_cast<float>(floodLightmA));
        }
    };
    if(boardName.find("PRO") != std::string::npos) {
        timer = node->create_wall_timer(500ms, cb);
    }

    #if defined(USE_CAMER_CONTROL)
        //------------------
        // set up camera
        // /home/nishi/local/git-download/depthai-core/examples/ColorCamera/rgb_camera_control.cpp
        // iso 1000  ->  sensIso
        // exposure, time: 27500  -> expTime
        //------------------
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

        std::cout << " Auto white-balance enable" << std::endl;
        ctrl.setAutoWhiteBalanceMode(dai::CameraControl::AutoWhiteBalanceMode::AUTO);
        controlQueue->send(ctrl);

    #endif

    //------------------
    // set up camera info
    //------------------

    //#define USE_ORG_CONVERTER
    #if defined(USE_ORG_CONVERTER)
        dai::rosBridge::ImageConverter depthConverter(tfPrefix + "_right_camera_optical_frame", true);
        dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", true);

        auto stereoCameraInfo = depthConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, monoWidth, monoHeight);
        auto previewCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, previewWidth, previewHeight);
        auto videoCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, colorWidth, colorHeight);
    #else
        camera_com::CameraTools camera_tools;
        camera_tools._reverseStereoSocketOrder = true;   // rtabmap_ros の rect_img のときは必要。

        auto stereoCameraInfo = camera_tools.calibrationToCameraInfo_my(calibrationHandler, dai::CameraBoardSocket::CAM_C, monoWidth, monoHeight);
        auto previewCameraInfo = camera_tools.calibrationToCameraInfo_my(calibrationHandler, dai::CameraBoardSocket::CAM_A, previewWidth, previewHeight);
        // test by nishi
        //colorWidth = colorHeight=416;
        auto videoCameraInfo = camera_tools.calibrationToCameraInfo_my(calibrationHandler, dai::CameraBoardSocket::CAM_A, colorWidth, colorHeight);
    #endif

    //std::cout << "Start while()" << std::endl;

    //bool im_ok=false;

    //----------------
    // start camera publisher
    //----------------
    if(useDepth) {
        go_depthQueue_pub.init(depthQueue);
        //go_depthQueue_pub.set_debug();
        go_depthQueue_pub.openPub(node, tfPrefix + "_right_camera_optical_frame", "stereo/depth", qos, stereoCameraInfo);
    }
    if(usePreview) {
        go_previewQueue_pub.init(previewQueue);
        //go_previewQueue_pub.set_debug();
        go_previewQueue_pub.openPub(node, tfPrefix + "_rgb_camera_optical_frame", "color/preview/image", qos, previewCameraInfo);
    }
    else{
        // yolo detection
        go_dpub_nNetDataQueue.init(nNetDataQueue);
        go_dpub_nNetDataQueue.set_labelmap(&labelMap);
        //go_dpub_nNetDataQueue.set_debug();
        go_dpub_nNetDataQueue.openPub_noInfo(node, tfPrefix + "_rgb_camera_optical_frame", "color/tiny_yolo_detections", qos, previewWidth, previewHeight, normalized);

    }
    if(useVideo) {
        // Center RGB Camera Video
        // 1920x1080 -> 640x480 にサイズ変更が必要。 by nishi 2024.6.22
        go_videoQueue_pub.init(videoQueue);
        //go_videoQueue_pub.set_debug();
        go_videoQueue_pub.rgb2grey_=rgb2grey;
        go_videoQueue_pub.set_resize();
        go_videoQueue_pub.openPub(node, tfPrefix + "_rgb_camera_optical_frame", "color/video/image", qos, videoCameraInfo,trace);
    }
    rclcpp::spin(node);
    //rclcpp::shutdown();

    return 0;
}