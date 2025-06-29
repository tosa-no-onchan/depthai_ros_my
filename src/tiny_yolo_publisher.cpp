/*
*
* depthai_ros_my/src/tiny_yolo_publisher.cpp
*
* original from
*  depthai-ros/depthai_examples/src/tiny_yolo_publisher.cpp
* reffer from
*  depthai-core/examples/MobileNet/tiny_yolo.cpp
*
* 1. build on SBC and PC
*  $ colcon build --symlink-install --parallel-workers 1 --packages-select depthai_ros_my
*  $ . install/setup.bash
*
* how to use ?
*  http://zdome.net/wiki/index.php/Object_Detection_Subscriber_for_Foxy_alsora_ros2-tensorflow_20220127
*/

#include <cstdio>
#include <iostream>

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImgDetectionConverter.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"

// add by nishi
#include "depthai_ros_my/camera_com.hpp"

#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// MobilenetSSD label texts
//static const std::vector<std::string> labelMap = {"background", "aeroplane", "bicycle",     "bird",  "boat",        "bottle", "bus",

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


dai::Pipeline createPipeline(bool syncNN, std::string nnPath, json &j_data,int rate=30) {
    dai::Pipeline pipeline;

    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();

    //auto detectionNetwork = pipeline.create<dai::node::MobileNetDetectionNetwork>();
    auto detectionNetwork = pipeline.create<dai::node::YoloDetectionNetwork>();
    auto nnOut = pipeline.create<dai::node::XLinkOut>();

    xlinkOut->setStreamName("preview");
    nnOut->setStreamName("detections");

    // Properties
    colorCam->setPreviewSize(416, 416);  // NN input
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    colorCam->setFps(rate);


    // testing tiny yolo DetectionNetwork
    detectionNetwork->setConfidenceThreshold(0.5f);

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

    // Link plugins CAM -> NN -> XLINK
    colorCam->preview.link(detectionNetwork->input);
    if(syncNN)
        detectionNetwork->passthrough.link(xlinkOut->input);
    else
        colorCam->preview.link(xlinkOut->input);        // colorCam->preview.link() は、2繋げられる?

    detectionNetwork->out.link(nnOut->input);
    return pipeline;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("tiny_yolo_node");

    std::cout << "tiny_yolo_publisher" << std::endl;

    std::string tfPrefix, resourceBaseFolder, nnPath;
    std::string cameraParamUri = "package://depthai_examples/params/camera";
    //std::string nnName(BLOB_NAME);
    std::string nnName = "yolov4_tiny_coco_416x416_openvino_2021.4_6shave_bgr.blob";
    bool syncNN=true;
    int bad_params = 0;

    int rate = 30;
    int queue_size = 2;
    bool normalized=true;

    std::string nnJson;
    json j_data;

    std::vector<std::string> labelMap;
    labelMap=labelMap_80;


    node->declare_parameter("tf_prefix", "oak");
    node->declare_parameter("camera_param_uri", cameraParamUri);
    node->declare_parameter("resourceBaseFolder", "");
    node->declare_parameter("sync_nn", syncNN);
    node->declare_parameter<std::string>("nnName", "");
    node->declare_parameter<std::string>("nnJson", "");

    node->declare_parameter("rate", rate);
    node->declare_parameter("queue_size", queue_size);
    node->declare_parameter("normalized", normalized);

    node->get_parameter("tf_prefix", tfPrefix);
    node->get_parameter("camera_param_uri", cameraParamUri);
    node->get_parameter("sync_nn", syncNN);
    node->get_parameter("resourceBaseFolder", resourceBaseFolder);

    if(resourceBaseFolder.empty()) {
        throw std::runtime_error("Send the path to the resouce folder containing NNBlob in \'resourceBaseFolder\' ");
    }
    // Uses the path from param if passed or else uses from BLOB_PATH from CMAKE
    std::string nnParam;
    node->get_parameter("nnName", nnParam);
    if(nnParam != "x") {
        node->get_parameter("nnName", nnName);
    }
    // add by nishi 2024.6.30
    node->get_parameter("nnJson", nnJson);

    node->get_parameter("rate", rate);
    node->get_parameter("queue_size", queue_size);
    node->get_parameter("normalized", normalized);

    std::cout << " nnName: "<< nnName << std::endl;
    std::cout << " rate: "<< rate << std::endl;
    std::cout << " queue_size: "<< queue_size << std::endl;


    nnPath = resourceBaseFolder + "/" + nnName;

    std::cout << "nnJson:"<< nnJson << std::endl;

    if(nnJson != ""){
        std::ifstream f(resourceBaseFolder + "/" + nnJson);
        try{
            j_data = json::parse(f);
            auto labels = j_data["mappings"]["labels"];
            labelMap = std::vector<std::string>(labels);
        }
        catch(...){
            std::cout << " nnJson access error"<< std::endl;
        }
    }

    //std::cout << "nnPath:"<< nnPath << std::endl;
    //std::cout << "resourceBaseFolder/nnJson:"<< resourceBaseFolder + "/" + nnJson << std::endl;


    dai::Pipeline pipeline = createPipeline(syncNN, nnPath, j_data, rate);
    dai::Device device(pipeline);

    std::shared_ptr<dai::DataOutputQueue> previewQueue = device.getOutputQueue("preview", queue_size, false);
    std::shared_ptr<dai::DataOutputQueue> nNetDataQueue = device.getOutputQueue("detections", queue_size, false);

    //#define USE_ORG_CONVERTER
    #if defined(USE_ORG_CONVERTER)
        std::string color_uri = cameraParamUri + "/" + "color.yaml";

        // TODO(sachin): Add option to use CameraInfo from EEPROM
        dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
        dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rgbPublish(
                previewQueue,
                node,
                std::string("color/image"),
                std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                            &rgbConverter,  // since the converter has the same frame name
                                            // and image type is also same we can reuse it
                            std::placeholders::_1,
                            std::placeholders::_2),
                30,
                color_uri,
                "color");

        dai::rosBridge::ImgDetectionConverter detConverter(tfPrefix + "_rgb_camera_optical_frame", 300, 300, false);
        dai::rosBridge::BridgePublisher<vision_msgs::msg::Detection2DArray, dai::ImgDetections> detectionPublish(
                nNetDataQueue,
                node,
                std::string("color/tiny_yolo_detections"),
                std::bind(&dai::rosBridge::ImgDetectionConverter::toRosMsg, &detConverter, std::placeholders::_1, std::placeholders::_2),
                30);

        detectionPublish.addPublisherCallback();
        rgbPublish.addPublisherCallback();  // addPublisherCallback works only when the dataqueue is non blocking.
    #else

        int previewWidth=416;
        int previewHeight=416;
        int qos=1;

        auto calibrationHandler = device.readCalibration();

        camera_com::CameraTools camera_tools;
        //camera_tools._reverseStereoSocketOrder = true;   // rtabmap_ros の rect_img のときは必要。

        auto previewCameraInfo = camera_tools.calibrationToCameraInfo_my(calibrationHandler, dai::CameraBoardSocket::CAM_A, previewWidth, previewHeight);

        camera_com::Go_Publish go_pub_previewQueue;
        camera_com::Go_DtectPublish go_dpub_nNetDataQueue;
        go_pub_previewQueue.init(previewQueue);
        //go_pub_previewQueue.set_debug();
        go_pub_previewQueue.openPub(node, tfPrefix + "_rgb_camera_optical_frame", "color/image", qos, previewCameraInfo);

        go_dpub_nNetDataQueue.init(nNetDataQueue);
        go_dpub_nNetDataQueue.set_labelmap(&labelMap);
        //go_dpub_nNetDataQueue.set_debug();
        go_dpub_nNetDataQueue.openPub_noInfo(node, tfPrefix + "_rgb_camera_optical_frame", "color/tiny_yolo_detections", qos, previewWidth, previewHeight, normalized);

    #endif

    rclcpp::spin(node);

    return 0;
}
