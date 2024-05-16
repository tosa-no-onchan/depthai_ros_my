/*
* stereo_camera.cpp
*
* depthai_ros_my/src/stereo_camera_test.cpp
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


std::tuple<dai::Pipeline, int, int, float> createPipeline(
    bool withDepth, bool lrcheck, bool extended, bool subpixel, int confidence, int LRchecktresh, std::string resolution,int rate,bool outputDepth,bool outputRectified) {

    // Create pipeline
    dai::Pipeline pipeline;
    dai::node::MonoCamera::Properties::SensorResolution monoResolution;

    //bool outputDepth=false;
    //bool outputRectified=true;

    std::cout << " createPipeline()" << std::endl;

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


    // XLinkOut
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");

    if(withDepth) {
        xoutDisp->setStreamName("disparity");
        xoutDepth->setStreamName("depth");
        xoutRectifL->setStreamName("rectified_left");
        xoutRectifR->setStreamName("rectified_right");
    }


    std::cout << " createPipeline() :#5 " << std::endl;

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
    //monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoLeft->setResolution(monoResolution);
    monoLeft->setCamera("left");
    //monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
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

    std::cout << " createPipeline() :#99" << std::endl;

    return std::make_tuple(pipeline, width, height, disparityMultiplier);
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("stereo_camera_my");

    std::cout << "start stereo_camera_my" << std::endl;

    bool outputDepth=false;
    bool outputRectified=true;


    std::string tfPrefix, mode, monoResolution;
    bool lrcheck, extended, subpixel, enableDepth;
    int confidence, LRchecktresh;
    int monoWidth, monoHeight;
    float disparityMultiplier;
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

    enableDepth = false;

    std::cout << " passed #2" << std::endl;

    std::tie(pipeline, monoWidth, monoHeight, disparityMultiplier) = createPipeline(enableDepth, lrcheck, extended, subpixel, confidence, LRchecktresh, monoResolution, rate,outputDepth,outputRectified);

    std::cout << " passed #2.1" << std::endl;

    dai::Device device(pipeline);

    std::cout << " passed #3" << std::endl;

    auto leftQueue = device.getOutputQueue("left", queue_size, false);
    auto rightQueue = device.getOutputQueue("right", queue_size, false);
    auto dispQueue = enableDepth ? device.getOutputQueue("disparity", queue_size, false) : nullptr;
    auto depthQueue = enableDepth ? device.getOutputQueue("depth", queue_size, false) : nullptr;

    auto rectifLeftQueue = enableDepth ? device.getOutputQueue("rectified_left", queue_size, false) : nullptr;
    auto rectifRightQueue = enableDepth ? device.getOutputQueue("rectified_right", queue_size, false) : nullptr;

    auto calibrationHandler = device.readCalibration();

    std::cout << "Start while()" << std::endl;

    bool im_ok=false;

    while(true) {

        //std::shared_ptr<dai::ImgFrame> left = leftQueue->get<dai::ImgFrame>();
        std::shared_ptr<dai::ImgFrame> left = leftQueue->tryGet<dai::ImgFrame>();
        if(left != nullptr){
            //printf("%s",left);

            //std::cout << "left->getWidth():" << left->getWidth() << std::endl;
            cv::Mat mat_left = cv::Mat(left->getData()).reshape(1,left->getHeight());


            //std::cout << "mat_left.cols:" << mat_left.cols << std::endl;
            //std::cout << "mat_left.rows:" << mat_left.rows << std::endl;
            cv::imshow("left", mat_left);
            im_ok=true;
        }

        //auto right = rightQueue->get<dai::ImgFrame>();
        auto right = rightQueue->tryGet<dai::ImgFrame>();
        if(right != nullptr){

            //std::cout << "right->getWidth():" << right->getWidth() << std::endl;

            cv::Mat mat_right = cv::Mat(right->getData()).reshape(1,right->getHeight());
            cv::imshow("right", mat_right);
            im_ok=true;
        }

        if(enableDepth){
            //auto disparity = dispQueue->get<dai::ImgFrame>();
            auto disparity = dispQueue->tryGet<dai::ImgFrame>();
            if(disparity != nullptr){
                std::cout << "disparity->getWidth():" << disparity->getWidth() << std::endl;
            }
            if(outputDepth){
                //auto depth = depthQueue->get<dai::ImgFrame>();
                auto depth = depthQueue->tryGet<dai::ImgFrame>();
                if(depth != nullptr){
                    std::cout << "depth->getWidth():" << depth->getWidth() << std::endl;
                }
            }

            if(outputRectified) {
                //auto rectifL = rectifLeftQueue->get<dai::ImgFrame>();
                auto rectifL = rectifLeftQueue->tryGet<dai::ImgFrame>();
                if (rectifL != nullptr){
                    std::cout << "rectifL->getWidth():" << rectifL->getWidth();
                    std::cout << " rectifL->getHeight():" << rectifL->getHeight() << std::endl;
                    //cv::imshow("rectified_left", rectifL->getFrame());
                }

                //auto rectifR = rectifRightQueue->get<dai::ImgFrame>();
                auto rectifR = rectifRightQueue->tryGet<dai::ImgFrame>();
                if (rectifR != nullptr){
                    //cv::imshow("rectified_right", rectifR->getFrame());
                }
            }
        }

        if(im_ok == true){
            int key = cv::waitKey(1);
            if(key == 'q' || key == 'Q') {
                return 0;
            }
        }
    }

    return 0;
}