/*
* depthai_ros_my/src/tiny_yolo.cpp
*
* original
*  /home/nishi/local/git-download/depthai-core/examples/Yolo/tiny_yolo.cpp
* reffer from
*  /home/nishi/local/git-download/depthai-core/examples/ColorCamera/rgb_video.cpp
*
* 1. build
*  $ colcon build --symlink-install --parallel-workers 1 --packages-select depthai_ros_my
*  $ . install/setup.bash
*
* 2. run
*  $ ros2 run depthai_ros_my tiny_yolo_v4 /home/nishi/colcon_ws/src/depthai_ros_my/resources/yolov4_tiny_coco_416x416_openvino_2021.4_6shave_bgr.blob
*
*  depthai-core original
*  $ ros2 run depthai_ros_my tiny_yolo_v4 /home/nishi/.hunter/_Base/PrivateData/d8d09b697dac298fe83cf8856740a21b1a61ab89/d8d09b6/raw/yolo-v4-tiny-tf_openvino_2021.4_6shave.blob
*  $ ros2 run depthai_ros_my tiny_yolo_v4 /home/nishi/.hunter/_Base/PrivateData/dedb2d4d96b23e42d15c15e454b8f02eca2713de/dedb2d4/raw/yolo-v3-tiny-tf_openvino_2021.4_6shave.blob
*
* Apend.
* You must use USB3 cable and Power supply enough. 
*/

#include <chrono>
#include <iostream>

#include "utility/utility.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

/*
The code is the same as for Tiny-yolo-V3, the only difference is the blob file.
The blob was compiled following this tutorial: https://github.com/TNTWEN/OpenVINO-YOLOV4
*/

static const std::vector<std::string> labelMap = {
    "person",        "bicycle",      "car",           "motorbike",     "aeroplane",   "bus",         "train",       "truck",        "boat",
    "traffic light", "fire hydrant", "stop sign",     "parking meter", "bench",       "bird",        "cat",         "dog",          "horse",
    "sheep",         "cow",          "elephant",      "bear",          "zebra",       "giraffe",     "backpack",    "umbrella",     "handbag",
    "tie",           "suitcase",     "frisbee",       "skis",          "snowboard",   "sports ball", "kite",        "baseball bat", "baseball glove",
    "skateboard",    "surfboard",    "tennis racket", "bottle",        "wine glass",  "cup",         "fork",        "knife",        "spoon",
    "bowl",          "banana",       "apple",         "sandwich",      "orange",      "broccoli",    "carrot",      "hot dog",      "pizza",
    "donut",         "cake",         "chair",         "sofa",          "pottedplant", "bed",         "diningtable", "toilet",       "tvmonitor",
    "laptop",        "mouse",        "remote",        "keyboard",      "cell phone",  "microwave",   "oven",        "toaster",      "sink",
    "refrigerator",  "book",         "clock",         "vase",          "scissors",    "teddy bear",  "hair drier",  "toothbrush"};

#define USE_ORG_NET

static std::atomic<bool> syncNN{true};
//static std::atomic<bool> syncNN{false};

int main(int argc, char** argv) {
    using namespace std;
    using namespace std::chrono;
    std::string nnPath(BLOB_PATH);


    // If path to blob specified, use that
    if(argc > 1) {
        nnPath = std::string(argv[1]);
    }

    // Print which blob we are using
    //printf("Using blob at path: %s\n", nnPath.c_str());
    std::cout << " Using blob at path:"<< nnPath << std::endl;

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    #if defined(USE_ORG_NET)
        auto detectionNetwork = pipeline.create<dai::node::YoloDetectionNetwork>();
    #endif

    // create xlink connections
    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
    #if defined(USE_ORG_NET)
        auto nnOut = pipeline.create<dai::node::XLinkOut>();
    #endif

    xoutRgb->setStreamName("rgb");
    #if defined(USE_ORG_NET)
        nnOut->setStreamName("detections");
    #endif


    // Properties
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setPreviewSize(416, 416);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setInterleaved(false);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    //camRgb->setFps(40);
    // changed by nishi 2024.5.19
    camRgb->setFps(30);

    #if defined(USE_ORG_NET)

        // Network specific settings
        detectionNetwork->setConfidenceThreshold(0.5f);
        detectionNetwork->setNumClasses(80);
        detectionNetwork->setCoordinateSize(4);
        detectionNetwork->setAnchors({10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319});
        detectionNetwork->setAnchorMasks({{"side26", {1, 2, 3}}, {"side13", {3, 4, 5}}});
        detectionNetwork->setIouThreshold(0.5f);
        detectionNetwork->setBlobPath(nnPath);
        detectionNetwork->setNumInferenceThreads(2);
        detectionNetwork->input.setBlocking(false);

        std::cout << " detectionNetwork->getNumClasses():"<< detectionNetwork->getNumClasses() << std::endl;

    #endif

    // Linking
    #if defined(USE_ORG_NET)
        camRgb->preview.link(detectionNetwork->input);
        if(syncNN) {
            detectionNetwork->passthrough.link(xoutRgb->input);
        } 
        else {
            camRgb->preview.link(xoutRgb->input);
        }
        detectionNetwork->out.link(nnOut->input);
    #else
        camRgb->preview.link(xoutRgb->input);
    #endif

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queues will be used to get the rgb frames and nn data from the outputs defined above
    auto qRgb = device.getOutputQueue("rgb", 4, false);
    #if defined(USE_ORG_NET)
        auto qDet = device.getOutputQueue("detections", 4, false);
    #endif

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
            cv::putText(frame, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
            std::stringstream confStr;
            confStr << std::fixed << std::setprecision(2) << detection.confidence * 100;
            cv::putText(frame, confStr.str(), cv::Point(x1 + 10, y1 + 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
            cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
        }
        // Show the frame
        cv::imshow(name, frame);
    };

    bool error_f=false;
    while(true) {
        std::shared_ptr<dai::ImgFrame> inRgb;
        #if defined(USE_ORG_NET)
            std::shared_ptr<dai::ImgDetections> inDet;
        #endif

        if(syncNN) {
            try{
                inRgb = qRgb->get<dai::ImgFrame>();
            }
            catch(...){
                std::cout << " inRgb = qRgb->get() error"<< std::endl;
                error_f=true;
            }
            #if defined(USE_ORG_NET)
                try{
                    inDet = qDet->get<dai::ImgDetections>();
                }
                catch(...){
                    std::cout << " inDet = qDet->get() error"<< std::endl;
                    error_f=true;
                }
            #endif
        } 
        else {
            try{
                inRgb = qRgb->tryGet<dai::ImgFrame>();
            }
            catch(...){
                std::cout << " inRgb = qRgb->tryGet() error"<< std::endl;
                error_f=true;
            }
            #if defined(USE_ORG_NET)
                try{
                    inDet = qDet->tryGet<dai::ImgDetections>();
                }
            catch(...){
                std::cout << " inDet = qDet->tryGet() error"<< std::endl;
                error_f=true;
            }
            #endif
        }
        if(error_f){
            std::cout << " Is USB 3.0 cable and the Power supply enough ?"<< std::endl;
            continue;
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

        #if defined(USE_ORG_NET)
            if(inDet) {
                detections = inDet->detections;
            }

            if(!frame.empty()) {
                displayFrame("rgb", frame, detections);
            }
        #else
            cv::imshow("frame", frame);
        #endif

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
