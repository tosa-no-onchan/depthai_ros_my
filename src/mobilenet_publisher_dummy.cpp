/*
* depthai_ros_my/src/mobilenet_publisher_dummy.cpp
*
* 1. build
*  $ colcon build --symlink-install --parallel-workers 1 --packages-select depthai_ros_my
*  $ . install/setup.bash
*
* 2. run
*   $ ros2 run depthai_ros_my mobilenet_publisher_dummy
*/

#include "depthai_ros_my/subs_com.hpp"


namespace camera_com {

class Go_DtectPublish_Dummy{
public:

    std::shared_ptr<rclcpp::Node> node_;
    bool rgb2grey_=false;

    bool debug_f_=false;

    Go_DtectPublish_Dummy(){}

    void openPub_noInfo(std::shared_ptr<rclcpp::Node> node, std::string frame_name, std::string topic_name, int qos,int width, int height , bool normalized, int rate){
        node_=node;
        frame_name_ = frame_name;
        topic_name_ = topic_name;
        width_=width;
        height_=height;
        normalized_=normalized;

        rclcpp::QoS video_qos(1);
        video_qos.reliability((rmw_qos_reliability_policy_t)qos);

        detect_pub_ = node_->create_publisher<vision_msgs::msg::Detection2DArray>(topic_name_, video_qos);

        rclcpp::WallRate loop(rate);

        while(rclcpp::ok()){
            loop.sleep();
            feedImages();
        }

    }

    void feedImages(){

        rclcpp::Time capture_time = node_->now();
        vision_msgs::msg::Detection2DArray opDetectionMsg;

        opDetectionMsg.header.stamp = capture_time;
        opDetectionMsg.header.frame_id = frame_name_;
        opDetectionMsg.detections.resize(1);

        for(int i = 0; i < 1 ; ++i) {

            float xMin, yMin, xMax, yMax;
            xMin = 0.1;
            xMax = 0.9;
            yMin = 0.5;
            yMax = 0.9;

            double confidence=0.7;
            int class_id=0;

            float xSize = xMax - xMin;
            float ySize = yMax - yMin;
            float xCenter = xMin + xSize / 2;
            float yCenter = yMin + ySize / 2;

            opDetectionMsg.detections[i].results.resize(1);

            #define IS_HUMBLE
            #if defined(IS_GALACTIC) || defined(IS_HUMBLE)
                opDetectionMsg.detections[i].id = std::to_string(0);
                //uint32_t labelIndex = inNetData->detections[i].label;
                //std::string labelStr = std::to_string(labelIndex);
                //if(is_labelMap_){
                //    // changed by nishi 2024.5.24
                //    if(labelIndex < labelMap_->size()){
                //        labelStr = labelMap_->at(labelIndex);
                //    }
                //}
                //opDetectionMsg.detections[i].id = labelStr;
                opDetectionMsg.detections[i].results[0].hypothesis.class_id = std::to_string(class_id);
                opDetectionMsg.detections[i].results[0].hypothesis.score = confidence;
            #elif defined(IS_ROS2)
                opDetectionMsg.detections[i].results[0].id = std::to_string(inNetData->detections[i].label);
                opDetectionMsg.detections[i].results[0].score = inNetData->detections[i].confidence;
            #endif
            #if defined(IS_HUMBLE)
                opDetectionMsg.detections[i].bbox.center.position.x = xCenter;
                opDetectionMsg.detections[i].bbox.center.position.y = yCenter;
            #else
                opDetectionMsg.detections[i].bbox.center.x = xCenter;
                opDetectionMsg.detections[i].bbox.center.y = yCenter;
            #endif
            opDetectionMsg.detections[i].bbox.size_x = xSize;
            opDetectionMsg.detections[i].bbox.size_y = ySize;
        }

        // exe publish
        detect_pub_->publish(opDetectionMsg);
        if(debug_f_){
            std::cout << " frame_name_:"<< frame_name_ << std::endl;
            //std::cout << " image->header.frame_id:"<< image->header.frame_id << std::endl;
        }

    }

  private:

    std::string frame_name_;
    std::string topic_name_;

    int width_, height_;
    bool normalized_;

    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detect_pub_;

};
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mobilenet_node_dummy");
    std::cout << "mobilenet_publisher_dummy" << std::endl;

    std::string tfPrefix="oak";

    int previewWidth=300;
    int previewHeight=300;
    int qos=1;
    bool normalized=true;

    int rate = 15;

    camera_com::Go_DtectPublish_Dummy go_dpub_dummy;
    go_dpub_dummy.openPub_noInfo(node, tfPrefix + "_rgb_camera_optical_frame", "color/mobilenet_detections", qos, previewWidth, previewHeight, normalized, rate);

    rclcpp::spin(node);
    return 0;

}