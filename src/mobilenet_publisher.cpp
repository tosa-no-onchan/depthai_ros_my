/*
*
* depthai_ros_my/src/mobilenet_publisher.cpp
*
* original from
*  depthai-ros/depthai_examples/src/mobilenet_publisher.cpp
* reffer from
*  depthai-core/examples/MobileNet/rgb_mobilenet.cpp
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

// MobilenetSSD label texts
//static const std::vector<std::string> labelMap = {"background", "aeroplane", "bicycle",     "bird",  "boat",        "bottle", "bus",
std::vector<std::string> labelMap = {"background", "aeroplane", "bicycle",     "bird",  "boat",        "bottle", "bus",
                                                  "car",        "cat",       "chair",       "cow",   "diningtable", "dog",    "horse",
                                                  "motorbike",  "person",    "pottedplant", "sheep", "sofa",        "train",  "tvmonitor"};


dai::Pipeline createPipeline(bool syncNN, std::string nnPath,int rate=30) {
    dai::Pipeline pipeline;
    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();
    auto detectionNetwork = pipeline.create<dai::node::MobileNetDetectionNetwork>();
    auto nnOut = pipeline.create<dai::node::XLinkOut>();

    xlinkOut->setStreamName("preview");
    nnOut->setStreamName("detections");

    // Properties
    colorCam->setPreviewSize(300, 300);  // NN input
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    colorCam->setFps(rate);

    // testing MobileNet DetectionNetwork
    detectionNetwork->setConfidenceThreshold(0.5f);
    detectionNetwork->setBlobPath(nnPath);

    // Link plugins CAM -> NN -> XLINK
    colorCam->preview.link(detectionNetwork->input);
    if(syncNN)
        detectionNetwork->passthrough.link(xlinkOut->input);
    else
        colorCam->preview.link(xlinkOut->input);        // ここ変?

    detectionNetwork->out.link(nnOut->input);
    return pipeline;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mobilenet_node");

    std::cout << "mobilenet_publisher" << std::endl;

    std::string tfPrefix, resourceBaseFolder, nnPath;
    std::string cameraParamUri = "package://depthai_examples/params/camera";
    std::string nnName(BLOB_NAME);
    bool syncNN;
    int bad_params = 0;

    int rate = 30;
    int queue_size = 2;
    bool normalized=true;

    node->declare_parameter("tf_prefix", "oak");
    node->declare_parameter("camera_param_uri", cameraParamUri);
    node->declare_parameter("resourceBaseFolder", "");
    node->declare_parameter("sync_nn", syncNN);
    node->declare_parameter<std::string>("nnName", "");

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

    node->get_parameter("rate", rate);
    node->get_parameter("queue_size", queue_size);
    node->get_parameter("normalized", normalized);

    std::cout << " nnName: "<< nnName << std::endl;
    std::cout << " rate: "<< rate << std::endl;
    std::cout << " queue_size: "<< queue_size << std::endl;


    nnPath = resourceBaseFolder + "/" + nnName;
    dai::Pipeline pipeline = createPipeline(syncNN, nnPath, rate);
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
            std::string("color/mobilenet_detections"),
            std::bind(&dai::rosBridge::ImgDetectionConverter::toRosMsg, &detConverter, std::placeholders::_1, std::placeholders::_2),
            30);

    detectionPublish.addPublisherCallback();
    rgbPublish.addPublisherCallback();  // addPublisherCallback works only when the dataqueue is non blocking.

    #else

        int previewWidth=300;
        int previewHeight=300;
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
        go_dpub_nNetDataQueue.openPub_noInfo(node, tfPrefix + "_rgb_camera_optical_frame", "color/mobilenet_detections", qos, previewWidth, previewHeight, normalized);

    #endif

    rclcpp::spin(node);

    return 0;
}
