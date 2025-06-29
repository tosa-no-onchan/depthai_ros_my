/*
* depthai_ros_my/src/camera_com.cpp
*/

#include "depthai_ros_my/camera_com.hpp"

namespace camera_com {

//------
// copy from src/depthai-ros/depthai_bridege/src/ImageConverter.cpp
//------
//std::unordered_map<dai::RawImgFrame::Type, std::string> Go_Publish::encodingEnumMap = {{dai::RawImgFrame::Type::YUV422i, "yuv422"},
std::unordered_map<dai::RawImgFrame::Type, std::string> encodingEnumMap = {{dai::RawImgFrame::Type::YUV422i, "yuv422"},
                                                                            {dai::RawImgFrame::Type::RGBA8888, "rgba8"},
                                                                            {dai::RawImgFrame::Type::RGB888i, "rgb8"},
                                                                            {dai::RawImgFrame::Type::BGR888i, "bgr8"},
                                                                            {dai::RawImgFrame::Type::GRAY8, "mono8"},
                                                                            {dai::RawImgFrame::Type::RAW8, "mono8"},
                                                                            {dai::RawImgFrame::Type::RAW16, "16UC1"},
                                                                            {dai::RawImgFrame::Type::YUV420p, "YUV420"}};
// TODO(sachin) : Move Planare to encodingEnumMap and use default planar namings. And convertt those that are not supported in ROS using ImageTransport in the
// bridge.
//std::unordered_map<dai::RawImgFrame::Type, std::string> Go_Publish::planarEncodingEnumMap = {
std::unordered_map<dai::RawImgFrame::Type, std::string> planarEncodingEnumMap = {
    {dai::RawImgFrame::Type::BGR888p, "rgb8"},  // 3_1_bgr8 represents 3 planes/channels and 1 byte per pixel in BGR format
    {dai::RawImgFrame::Type::RGB888p, "rgb8"},
    {dai::RawImgFrame::Type::NV12, "rgb8"},
    {dai::RawImgFrame::Type::YUV420p, "rgb8"}};

//-----
// define myself from /usr/local/include/depthai-shared/datatype/RawImgFrame.hpp
//-----
std::unordered_map<dai::RawImgFrame::Type, std::string> img_typeMap = {
        {dai::RawImgFrame::Type::YUV422i,"YUV422i"},    // interleaved 8 bit
        {dai::RawImgFrame::Type::YUV444p,"YUV444p"},    // planar 4:4:4 format
        {dai::RawImgFrame::Type::YUV420p,"YUV420p"},    // planar 4:2:0 format
        {dai::RawImgFrame::Type::YUV422p,"YUV422p"},    // planar 8 bit
        {dai::RawImgFrame::Type::YUV400p,"YUV400p"},    // 8-bit greyscale
        {dai::RawImgFrame::Type::RGBA8888,"RGBA8888"},   // RGBA interleaved stored in 32 bit word
        {dai::RawImgFrame::Type::RGB161616,"RGB161616"},  // Planar 16 bit RGB data
        {dai::RawImgFrame::Type::RGB888p,"RGB888p"},    // Planar 8 bit RGB data
        {dai::RawImgFrame::Type::BGR888p,"BGR888p"},    // Planar 8 bit BGR data
        {dai::RawImgFrame::Type::RGB888i,"RGB888i"},    // Interleaved 8 bit RGB data
        {dai::RawImgFrame::Type::BGR888i,"BGR888i"},    // Interleaved 8 bit BGR data
        {dai::RawImgFrame::Type::LUT2,"LUT2"},       // 1 bit  per pixel, Lookup table (used for graphics layers)
        {dai::RawImgFrame::Type::LUT4,"LUT4"},       // 2 bits per pixel, Lookup table (used for graphics layers)
        {dai::RawImgFrame::Type::LUT16,"LUT16"},      // 4 bits per pixel, Lookup table (used for graphics layers)
        {dai::RawImgFrame::Type::RAW16,"RAW16"},      // save any raw type (8, 10, 12bit) on 16 bits
        {dai::RawImgFrame::Type::RAW14,"RAW14"},      // 14bit value in 16bit storage
        {dai::RawImgFrame::Type::RAW12,"RAW12"},      // 12bit value in 16bit storage
        {dai::RawImgFrame::Type::RAW10,"RAW10"},      // 10bit value in 16bit storage
        {dai::RawImgFrame::Type::RAW8,"RAW8"},
        {dai::RawImgFrame::Type::PACK10,"PACK10"},  // SIPP 10bit packed format
        {dai::RawImgFrame::Type::PACK12,"PACK12"},  // SIPP 12bit packed format
        {dai::RawImgFrame::Type::YUV444i,"YUV444i"},
        {dai::RawImgFrame::Type::NV12,"NV12"},
        {dai::RawImgFrame::Type::NV21,"NV21"},
        {dai::RawImgFrame::Type::BITSTREAM,"BITSTREAM"},  // used for video encoder bitstream
        {dai::RawImgFrame::Type::HDR,"HDR"},
        {dai::RawImgFrame::Type::RGBF16F16F16p,"RGBF16F16F16p"},  // Planar FP16 RGB data
        {dai::RawImgFrame::Type::BGRF16F16F16p,""},  // Planar FP16 BGR data
        {dai::RawImgFrame::Type::RGBF16F16F16i,""},  // Interleaved FP16 RGB data
        {dai::RawImgFrame::Type::BGRF16F16F16i,""},  // Interleaved FP16 BGR data
        {dai::RawImgFrame::Type::GRAY8,"GRAY8"},          // 8 bit grayscale (1 plane)
        {dai::RawImgFrame::Type::GRAYF16,"GRAYF16"},        // FP16 grayscale (normalized)
        {dai::RawImgFrame::Type::NONE,"NONE"}};


//------
// CameraTools::calibrationToCameraInfo_my()
// orginal
//  src/depthai-ros/depthai_bridge/src/ImageConverter.cpp
//    ImageConverter::calibrationToCameraInfo()
//------
sensor_msgs::msg::CameraInfo CameraTools::calibrationToCameraInfo_my(dai::CalibrationHandler calibHandler, dai::CameraBoardSocket cameraId, int width, int height, dai::Point2f topLeftPixelId, dai::Point2f bottomRightPixelId){
    std::vector<std::vector<float>> camIntrinsics, rectifiedRotation;
    std::vector<float> distCoeffs;
    std::vector<double> flatIntrinsics, distCoeffsDouble;
    int defWidth, defHeight;
    sensor_msgs::msg::CameraInfo cameraData;
    std::tie(std::ignore, defWidth, defHeight) = calibHandler.getDefaultIntrinsics(cameraId);

    if(width == -1) {
        cameraData.width = static_cast<uint32_t>(defWidth);
    } else {
        cameraData.width = static_cast<uint32_t>(width);
    }

    if(height == -1) {
        cameraData.height = static_cast<uint32_t>(defHeight);
    } else {
        cameraData.height = static_cast<uint32_t>(height);
    }

    camIntrinsics = calibHandler.getCameraIntrinsics(cameraId, cameraData.width, cameraData.height, topLeftPixelId, bottomRightPixelId);

    flatIntrinsics.resize(9);
    for(int i = 0; i < 3; i++) {
        std::copy(camIntrinsics[i].begin(), camIntrinsics[i].end(), flatIntrinsics.begin() + 3 * i);
    }

    auto& intrinsics = cameraData.k;
    auto& distortions = cameraData.d;
    auto& projection = cameraData.p;
    auto& rotation = cameraData.r;
    // Set rotation to reasonable default even for non-stereo pairs
    rotation[0] = rotation[4] = rotation[8] = 1;
    for(size_t i = 0; i < 3; i++) {
        std::copy(flatIntrinsics.begin() + i * 3, flatIntrinsics.begin() + (i + 1) * 3, projection.begin() + i * 4);
    }
    std::copy(flatIntrinsics.begin(), flatIntrinsics.end(), intrinsics.begin());

    distCoeffs = calibHandler.getDistortionCoefficients(cameraId);

    for(size_t i = 0; i < 8; i++) {
        distortions.push_back(static_cast<double>(distCoeffs[i]));
    }

    // Setting Projection matrix if the cameras are stereo pair. Right as the first and left as the second.
    if(calibHandler.getStereoRightCameraId() != dai::CameraBoardSocket::AUTO && calibHandler.getStereoLeftCameraId() != dai::CameraBoardSocket::AUTO) {
        if(calibHandler.getStereoRightCameraId() == cameraId || calibHandler.getStereoLeftCameraId() == cameraId) {
            std::vector<std::vector<float>> stereoIntrinsics = calibHandler.getCameraIntrinsics(
                calibHandler.getStereoRightCameraId(), cameraData.width, cameraData.height, topLeftPixelId, bottomRightPixelId);

            if(_alphaScalingEnabled) {
                cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64F);
                for(int i = 0; i < 3; i++) {
                    for(int j = 0; j < 3; j++) {
                        cameraMatrix.at<double>(i, j) = stereoIntrinsics[i][j];
                    }
                }
                cv::Mat distCoefficients(distCoeffs);

                cv::Mat newCameraMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoefficients, cv::Size(width, height), _alphaScalingFactor);
                // Copying the contents of newCameraMatrix to stereoIntrinsics
                for(int i = 0; i < 3; i++) {
                    for(int j = 0; j < 3; j++) {
                        float newValue = static_cast<float>(newCameraMatrix.at<double>(i, j));
                        stereoIntrinsics[i][j] = newValue;
                        intrinsics[i * 3 + j] = newValue;
                    }
                }
            }
            std::vector<double> stereoFlatIntrinsics(12), flatRectifiedRotation(9);
            for(int i = 0; i < 3; i++) {
                std::copy(stereoIntrinsics[i].begin(), stereoIntrinsics[i].end(), stereoFlatIntrinsics.begin() + 4 * i);
                stereoFlatIntrinsics[(4 * i) + 3] = 0;
            }

            // Check stereo socket order
            dai::CameraBoardSocket stereoSocketFirst = calibHandler.getStereoLeftCameraId();
            dai::CameraBoardSocket stereoSocketSecond = calibHandler.getStereoRightCameraId();
            double factor = 1.0;
            if(_reverseStereoSocketOrder) {
                stereoSocketFirst = calibHandler.getStereoRightCameraId();
                stereoSocketSecond = calibHandler.getStereoLeftCameraId();
                factor = -1.0;
            }

            if(stereoSocketFirst == cameraId) {
                // This defines where the first camera is w.r.t second camera coordinate system giving it a translation to place all the points in the first
                // camera to second camera by multiplying that translation vector using transformation function.
                stereoFlatIntrinsics[3] = factor * stereoFlatIntrinsics[0] * calibHandler.getCameraExtrinsics(stereoSocketFirst, stereoSocketSecond)[0][3]
                                          / 100.0;  // Converting to meters
                rectifiedRotation = calibHandler.getStereoLeftRectificationRotation();
            } else {
                rectifiedRotation = calibHandler.getStereoRightRectificationRotation();
            }

            for(int i = 0; i < 3; i++) {
                std::copy(rectifiedRotation[i].begin(), rectifiedRotation[i].end(), flatRectifiedRotation.begin() + 3 * i);
            }

            std::copy(stereoFlatIntrinsics.begin(), stereoFlatIntrinsics.end(), projection.begin());
            std::copy(flatRectifiedRotation.begin(), flatRectifiedRotation.end(), rotation.begin());
        }
    }
    cameraData.distortion_model = "rational_polynomial";

    return cameraData;
}


/*
* classs Go_Publish members
*/
/*--------------
* Go_Publish::openPub()
----------------*/
void Go_Publish::openPub(std::shared_ptr<rclcpp::Node> node, std::string frame_name, std::string topic_name, int qos, sensor_msgs::msg::CameraInfo &cameraInfo,bool trace){
    node_=node;
    frame_name_ = frame_name;
    topic_name_ = topic_name;
    trace_ = trace; 

    if(debug_f_)
        std::cout << "openPub() start" << std::endl;

    info_mgr_ = std::make_shared<camera_info_manager::CameraInfoManager>(node_.get());
    info_mgr_->setCameraInfo(cameraInfo);

    // add by nishi 22024.6.22
    height_=cameraInfo.height;
    width_=cameraInfo.width;

    //#define TEST_KKX
    //#if defined(TEST_KKX)
    rmw_qos_profile_t video_qos_profile = rmw_qos_profile_sensor_data;
    video_qos_profile.depth=1;
    switch(qos){
        case 0:
            video_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
        break;
        case 1:
            video_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        break;
        case 2:
            video_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        break;
    }
    //video_qos_profile.reliability = (rmw_qos_reliability_policy_t)qos;

    //video_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    video_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;

    camera_transport_pub_ = image_transport::create_camera_publisher(node_.get(), topic_name_, video_qos_profile);

    //#endif

    // rate trace ON ?
    if(trace_){
        start_t_ = node_->now();
    }

    Que_Recv::pub_ready_=true;

    if(debug_f_)
        std::cout << "openPub(): 99" << std::endl;
}

/*--------------
* Go_Publish::openPub_noInfo()
----------------*/
void Go_Publish::openPub_noInfo(std::shared_ptr<rclcpp::Node> node, std::string frame_name, std::string topic_name, int qos){

    node_=node;
    frame_name_ = frame_name;
    topic_name_ = topic_name;

    Que_Recv::pub_ready_=true;

    noInfo_f_=true;

}

/*--------------
* Go_Publish::feedImages()
----------------*/
void Go_Publish::feedImages(std::shared_ptr<dai::ADatatype> &Data){
    if(debug_f_){
        std::cout << "feedImagesPub() name:=" << Que_Recv::name_ << std::endl;
        std::cout << " frame_name_:"<< frame_name_ << std::endl;
    }
    auto inData = std::dynamic_pointer_cast<dai::ImgFrame>(Data);

    rclcpp::Time capture_time = node_->now();
    cv::Mat mat, output;

    //std::cout << " inData->getType()=" << inData->getType() << std::endl;
    //printf("%s",inData->getType());

    //ImageMsgs::Image outImageMsg;
    //StdMsgs::Header header;
    //header.frame_id = _frameName;

    cv::Size size = {0, 0};
    int type = 0;

    std::shared_ptr<sensor_msgs::msg::Image> image = std::make_shared<sensor_msgs::msg::Image>();
    //auto image = std::make_shared<sensor_msgs::msg::Image>();

    image->header.stamp = capture_time;
    image->header.frame_id = frame_name_;

    //-----
    // these code's base is from /depthai-ros/depthai_bridege/src/ImageConverter::toRosMsgRawPtr()
    //-----
    if(debug_f_){
        try{
            std::cout<< " inData->getType():" << img_typeMap.at(inData->getType()) << std::endl;
        }
        catch(...){
            std::cout<< " inData->getType(): miss match" << std::endl;
        }
    }

    if(planarEncodingEnumMap.find(inData->getType()) != planarEncodingEnumMap.end()) {
        if(debug_f_)
            std::cout << " toRosMsgRawPtr(): #2" << std::endl;

        switch(inData->getType()) {
            case dai::RawImgFrame::Type::BGR888p:
            case dai::RawImgFrame::Type::RGB888p:
                if(debug_f_)
                    std::cout << " BGR888p or RGB888p" << std::endl;
                size = cv::Size(inData->getWidth(), inData->getHeight());
                type = CV_8UC3;
                break;
            case dai::RawImgFrame::Type::YUV420p:
            case dai::RawImgFrame::Type::NV12:
                if(debug_f_)
                    std::cout << " YUV420p or NV12" << std::endl;
                size = cv::Size(inData->getWidth(), inData->getHeight() * 3 / 2);
                type = CV_8UC1;
                break;

            default:
                if(debug_f_){
                    //std::runtime_error("Invalid dataType inputs..");
                    std::cout << " Invalid dataType inputs.." << std::endl;
                }
                break;
        }
        mat = cv::Mat(size, type, inData->getData().data());

        //#define TEST_VIEW_1
        #if defined(TEST_VIEW_1)
            cv::imshow(Que_Recv::name_, mat);

            int key = cv::waitKey(1);
            if(key == 'q' || key == 'Q') {
                return;
            }
        #endif

        auto encoding_code = sensor_msgs::image_encodings::BGR8;

        switch(inData->getType()) {
            case dai::RawImgFrame::Type::RGB888p: {
                if(debug_f_)
                    std::cout << " dai::RawImgFrame::Type::RGB888p" << std::endl;
                cv::Size s(inData->getWidth(), inData->getHeight());
                cv::Mat m1 = cv::Mat(s, CV_8UC1, inData->getData().data() + s.area() * 2);
                cv::Mat m2 = cv::Mat(s, CV_8UC1, inData->getData().data() + s.area() * 1);
                cv::Mat m3 = cv::Mat(s, CV_8UC1, inData->getData().data() + s.area() * 0);
                cv::Mat channels[3] = {m1, m2, m3};
                cv::merge(channels, 3, output);
            } break;

            case dai::RawImgFrame::Type::BGR888p: {
                if(debug_f_)
                    std::cout << " dai::RawImgFrame::Type::BGR888p" << std::endl;
                cv::Size s(inData->getWidth(), inData->getHeight());
                cv::Mat m1 = cv::Mat(s, CV_8UC1, inData->getData().data() + s.area() * 0);
                cv::Mat m2 = cv::Mat(s, CV_8UC1, inData->getData().data() + s.area() * 1);
                cv::Mat m3 = cv::Mat(s, CV_8UC1, inData->getData().data() + s.area() * 2);
                cv::Mat channels[3] = {m1, m2, m3};
                cv::merge(channels, 3, output);
            } break;

            case dai::RawImgFrame::Type::YUV420p:
                if(debug_f_)
                    std::cout << " dai::RawImgFrame::Type::YUV420p" << std::endl;
                cv::cvtColor(mat, output, cv::ColorConversionCodes::COLOR_YUV2BGR_IYUV);
                break;

            // center rgb Camera
            case dai::RawImgFrame::Type::NV12:
                if(debug_f_)
                    std::cout << " dai::RawImgFrame::Type::NV12" << std::endl;
                // add by nishi 2024.5.17
                if(rgb2grey_){
                    // bgr to grey
                    // COLOR_YUV2GRAY_NV12
                    cv::cvtColor(mat, output, cv::ColorConversionCodes::COLOR_YUV2GRAY_NV12);
                    encoding_code = sensor_msgs::image_encodings::MONO8;
                }
                else{
                    // original  to BGR8
                    //cv::cvtColor(mat, output, cv::ColorConversionCodes::COLOR_YUV2BGR_NV12);
                    // changed by nishi 2024.5.18
                    // to RGB8
                    cv::cvtColor(mat, output, cv::ColorConversionCodes::COLOR_YUV2RGB_NV12);
                    encoding_code = sensor_msgs::image_encodings::RGB8;
                }
                if(is_resize_ && (height_ != output.rows || width_ != output.cols)){
                    //リサイズ
                    //output = resizeKeepAspectRatio(output, cv::Size(width_,height_), cv::Scalar(0));
                    output = resizeCutOverEdge(output, cv::Size(width_,height_));

                }
                break;

            default:
                if(debug_f_)
                    std::cout << " dai::RawImgFrame::Type::default" << std::endl;
                //output = mat.clone();
                output = mat;
                break;
        }

        #define USE_ORG_1
        #if defined(USE_ORG_1)
            // http://docs.ros.org/en/lunar/api/cv_bridge/html/c++/classcv__bridge_1_1CvImage.html
            //cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, output).toImageMsg(outImageMsg);
            //image = cv_bridge::CvImage(image->header, sensor_msgs::image_encodings::BGR8, output).toImageMsg();
            image = cv_bridge::CvImage(image->header, encoding_code, output).toImageMsg();
        #else
            image->height=inData->getHeight();
            image->width= inData->getWidth();
            image->step = image->width;
            image->encoding = sensor_msgs::image_encodings::MONO8;
            //image->header.stamp = capture_time;
            //image->header.frame_id = frame_name_;

            image->data.resize(image->step * image->height);

            //std::cout << " image->height=" << image->height <<" image->width="<< image->width << std::endl;

            memcpy((unsigned char *)(&image->data[0]),output.data, image->step * image->height);
        #endif
    }
    // raw , rect ,depth, disparity は、こちらか!!
    else if(encodingEnumMap.find(inData->getType()) != encodingEnumMap.end()) {
        // rect data
        if(debug_f_)
            std::cout << " toRosMsgRawPtr(): #3" << std::endl;

        // copying the data to ros msg
        //outImageMsg.header = header;
        std::string temp_str(encodingEnumMap[inData->getType()]);
        //outImageMsg.encoding = temp_str;
        image->encoding = temp_str; 

        if(debug_f_)
            std::cout << " temp_str:"<< temp_str << std::endl;

        //outImageMsg.height = inData->getHeight();
        image->height = inData->getHeight();
        image->width = inData->getWidth();
        image->step = inData->getData().size() / inData->getHeight();

        // test by nishi 2024.5.11
        //std::cout << " outImageMsg.encoding:"<< outImageMsg.encoding << std::endl;

        //if(image->encoding == "16UC1"){
        // changed by nishi
        if(image->encoding == "16UC1" || image->encoding == "mono8"){
            image->is_bigendian = false;
        }
        else
            image->is_bigendian = true;

        size_t size = inData->getData().size();
        image->data.reserve(size);
        image->data = std::move(inData->getData());

    }
    if(trace_){
        cnt_++;
        rclcpp::Duration elapsed = capture_time - start_t_;
        if(elapsed.seconds() >= sec_dur_){
            std::cout << " oak-d_camera: "<< Que_Recv::name_ << " rate: " << cnt_/(int)sec_dur_ << std::endl;
            start_t_ = capture_time;
            cnt_=0;
        }
    }

    if(debug_f_){
        std::cout << " frame_name_:"<< frame_name_ << std::endl;
        std::cout << " image->header.frame_id:"<< image->header.frame_id << std::endl;
    }
    if(noInfo_f_!=true)
        sendInfo(capture_time, image);

}


/*--------------
* Go_Publish::sendInfo() for image
----------------*/
void Go_Publish::sendInfo(rclcpp::Time time, std::shared_ptr<sensor_msgs::msg::Image> const & img) {

    if(debug_f_)
        std::cout << " called Go_Publish::sendInfo()" << std::endl;

    std::shared_ptr<sensor_msgs::msg::CameraInfo> info = std::make_shared<sensor_msgs::msg::CameraInfo>(info_mgr_->getCameraInfo());
    //printf("%s",info_left);

    if (!checkCameraInfo(*img, *info)) {
        *info = sensor_msgs::msg::CameraInfo{};
        info->height = img->height;
        info->width = img->width;
    }

    info->header.frame_id = frame_name_;
    info->header.stamp = time;
    // add by nishi 2024.5.4
    //img->header.stamp = time;

    //trace_sts_=20;

    //if(intra_){
    //    left_pub_->publish(*img_l);
    //    left_info_pub_->publish(*info_left);
    //}
    //else{
        // http://docs.ros.org/en/jade/api/image_transport/html/classimage__transport_1_1CameraPublisher.html
        // https://github.com/ros-perception/image_transport_tutorials
        camera_transport_pub_.publish(img, info);

    //trace_sts_=23;
    //}
}

}