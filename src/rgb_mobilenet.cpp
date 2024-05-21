/*
* depthai_ros_my/src/rgb_mobilenet.cpp
*
* original from
* depthai-core/examples/MobileNet/rgb_mobilenet.cpp
*
* 1. build
*  $ colcon build --symlink-install --parallel-workers 1 --packages-select depthai_ros_my
* $ . install/setup.bash
*
* 2. run
*  $ ros2 run depthai_ros_my rgb_mobilenet /home/nishi/colcon_ws/src/depthai_ros_my/resources/mobilenet-ssd_openvino_2021.2_6shave.blob
*
*  depthai-cor original
*  $ ros2 run depthai_ros_my rgb_mobilenet /home/nishi/.hunter/_Base/PrivateData/4f4506726e3083981064938a0faaf9af6180d2c6/4f45067/raw/mobilenet-ssd_openvino_2021.4_6shave.blob
*/

#include <chrono>
#include <cstdio>
#include <iostream>

#include "utility/utility.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

// MobilenetSSD label texts
static const std::vector<std::string> labelMap = {"background", "aeroplane", "bicycle",     "bird",  "boat",        "bottle", "bus",
                                                  "car",        "cat",       "chair",       "cow",   "diningtable", "dog",    "horse",
                                                  "motorbike",  "person",    "pottedplant", "sheep", "sofa",        "train",  "tvmonitor"};

static std::atomic<bool> syncNN{true};

int main(int argc, char** argv) {
    using namespace std;
    using namespace std::chrono;
    // Default blob path provided by Hunter private data download
    // Applicable for easier example usage only
    std::string nnPath(BLOB_PATH);

    // If path to blob specified, use that
    if(argc > 1) {
        nnPath = std::string(argv[1]);
    }

    // Print which blob we are using
    printf("Using blob at path: %s\n", nnPath.c_str());

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto nn = pipeline.create<dai::node::MobileNetDetectionNetwork>();

    // create xlink connections
    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
    auto nnOut = pipeline.create<dai::node::XLinkOut>();
    auto nnNetworkOut = pipeline.create<dai::node::XLinkOut>();

    xoutRgb->setStreamName("rgb");
    nnOut->setStreamName("nn");
    nnNetworkOut->setStreamName("nnNetwork");

    // Properties
    camRgb->setPreviewSize(300, 300);  // NN input
    camRgb->setInterleaved(false);
    camRgb->setFps(40);
    // Define a neural network that will make predictions based on the source frames
    nn->setConfidenceThreshold(0.5);
    nn->setBlobPath(nnPath);
    nn->setNumInferenceThreads(2);
    nn->input.setBlocking(false);

    // Linking
    if(syncNN) {
        nn->passthrough.link(xoutRgb->input);
    } else {
        camRgb->preview.link(xoutRgb->input);
    }

    camRgb->preview.link(nn->input);
    nn->out.link(nnOut->input);
    nn->outNetwork.link(nnNetworkOut->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queues will be used to get the rgb frames and nn data from the outputs defined above
    auto qRgb = device.getOutputQueue("rgb", 4, false);
    auto qDet = device.getOutputQueue("nn", 4, false);
    auto qNN = device.getOutputQueue("nnNetwork", 4, false);

    cv::Mat frame;
    std::vector<dai::ImgDetection> detections;
    auto startTime = steady_clock::now();
    int counter = 0;
    float fps = 0;
    auto color2 = cv::Scalar(255, 255, 255);

    // Add bounding boxes and text to the frame and show it to the user
    auto displayFrame = [](std::string name, cv::Mat frame, std::vector<dai::ImgDetection>& detections) {
        auto color = cv::Scalar(255, 0, 0);
        // nn data, being the bounding box locations, are in <0..1> range - they need to be normalized with frame width/height
        for(auto& detection : detections) {
            int x1 = detection.xmin * frame.cols;
            int y1 = detection.ymin * frame.rows;
            int x2 = detection.xmax * frame.cols;
            int y2 = detection.ymax * frame.rows;

            uint32_t labelIndex = detection.label;
            std::string labelStr = to_string(labelIndex);
            if(labelIndex < labelMap.size()) {
                labelStr = labelMap[labelIndex];
            }
            cv::putText(frame, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            std::stringstream confStr;
            confStr << std::fixed << std::setprecision(2) << detection.confidence * 100;
            cv::putText(frame, confStr.str(), cv::Point(x1 + 10, y1 + 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
        }
        // Show the frame
        cv::imshow(name, frame);
    };

    bool printOutputLayersOnce = true;

    while(true) {
        std::shared_ptr<dai::ImgFrame> inRgb;
        std::shared_ptr<dai::ImgDetections> inDet;
        std::shared_ptr<dai::NNData> inNN;

        if(syncNN) {
            inRgb = qRgb->get<dai::ImgFrame>();
            inDet = qDet->get<dai::ImgDetections>();
            inNN = qNN->get<dai::NNData>();
        } else {
            inRgb = qRgb->tryGet<dai::ImgFrame>();
            inDet = qDet->tryGet<dai::ImgDetections>();
            inNN = qNN->tryGet<dai::NNData>();
        }

        counter++;
        auto currentTime = steady_clock::now();
        auto elapsed = duration_cast<duration<float>>(currentTime - startTime);
        if(elapsed > seconds(1)) {
            fps = counter / elapsed.count();
            counter = 0;
            startTime = currentTime;
        }

        if(inRgb) {
            // 後でセットします。
            //frame = inRgb->getCvFrame();
            //---
            cv::Size size = {0, 0};
            int type = 0;
            bool debug_f_=false;
            switch(inRgb->getType()) {
                case dai::RawImgFrame::Type::BGR888p:{
                    if(debug_f_)
                        std::cout << " BGR888p" << std::endl;
                    cv::Size s(inRgb->getWidth(), inRgb->getHeight());
                    cv::Mat m1 = cv::Mat(s, CV_8UC1, inRgb->getData().data() + s.area() * 0);
                    cv::Mat m2 = cv::Mat(s, CV_8UC1, inRgb->getData().data() + s.area() * 1);
                    cv::Mat m3 = cv::Mat(s, CV_8UC1, inRgb->getData().data() + s.area() * 2);
                    cv::Mat channels[3] = {m1, m2, m3};
                    cv::merge(channels, 3, frame);

                } break;

                case dai::RawImgFrame::Type::RGB888p:{
                    if(debug_f_)
                        std::cout << " dai::RawImgFrame::Type::RGB888p" << std::endl;
                    cv::Size s(inRgb->getWidth(), inRgb->getHeight());
                    cv::Mat m1 = cv::Mat(s, CV_8UC1, inRgb->getData().data() + s.area() * 2);
                    cv::Mat m2 = cv::Mat(s, CV_8UC1, inRgb->getData().data() + s.area() * 1);
                    cv::Mat m3 = cv::Mat(s, CV_8UC1, inRgb->getData().data() + s.area() * 0);
                    cv::Mat channels[3] = {m1, m2, m3};
                    cv::merge(channels, 3, frame);

                }
                    break;
                case dai::RawImgFrame::Type::YUV420p:
                case dai::RawImgFrame::Type::NV12:
                    if(debug_f_)
                        std::cout << " YUV420p or NV12" << std::endl;
                    size = cv::Size(inRgb->getWidth(), inRgb->getHeight() * 3 / 2);
                    type = CV_8UC1;
                    frame = cv::Mat(size, type, inRgb->getData().data());
                    break;

                default:
                    if(debug_f_){
                        //std::runtime_error("Invalid dataType inputs..");
                        std::cout << " Invalid dataType inputs.." << std::endl;
                    }
                    break;
            }

            //#define TEST_VIEW_1
            #if defined(TEST_VIEW_1)
                cv::imshow("frame", frame);

                //int key = cv::waitKey(1);
                //if(key == 'q' || key == 'Q') {
                //    return 0;
                //}
            #endif
            //----

            std::stringstream fpsStr;
            fpsStr << "NN fps: " << std::fixed << std::setprecision(2) << fps;
            cv::putText(frame, fpsStr.str(), cv::Point(2, inRgb->getHeight() - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color2);
        }

        if(inDet) {
            detections = inDet->detections;
        }

        if(printOutputLayersOnce && inNN) {
            std::cout << "Output layer names: ";
            for(const auto& ten : inNN->getAllLayerNames()) {
                std::cout << ten << ", ";
            }
            std::cout << std::endl;
            printOutputLayersOnce = false;
        }

        if(!frame.empty()) {
            displayFrame("video", frame, detections);
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
