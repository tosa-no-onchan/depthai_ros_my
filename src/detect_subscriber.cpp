/*
* detect_subscriber.cpp
*
* depthai_ros_my/src/detect_subscriber.cpp
*
* 1. build
*  $ colcon build --symlink-install --parallel-workers 1 --packages-select depthai_ros_my
*  $ . install/setup.bash
*
* 2. run
*   $ ros2 run depthai_ros_my detect_subscriber
*
* reffer from
*   http://rpgincpp.cocolog-nifty.com/blog/2006/02/post_0123.html
*   https://cpprefjp.github.io/lang/cpp23/class_template_argument_deduction_from_inherited.html
*
*/

//#include "depthai/depthai.hpp"

#include "depthai_ros_my/subs_com.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

namespace subs_com{

//-------------------
// class Detect_Subs
//-------------------
//template <class RosMsg> class Detect_Subs: public Subs_Com<RosMsg>{
class Detect_Subs: public Subs_Com<sensor_msgs::msg::Image>{
//using Subs_Com<RosMsg>::Subs_Com;
public:
    Detect_Subs(){}
    void activate_image(std::shared_ptr<rclcpp::Node> node, std::string topic, int queue_size){
        node_=node;
        topic_=topic;
        if(is_trace)
            std::cout << " activate_image() topic: "<< topic_ << std::endl;

        //Subs_Com<RosMsg>::init(node_,topic_, queue_size);   // OK
        Subs_Com<sensor_msgs::msg::Image>::init(node_,topic_, queue_size);   // OK
        //this->init(node_,topic_,queue_size);      // OK
    }

    void activate_detect(std::string topic, int queue_size=10){
        //dtect_sub_ = node_->create_subscription<vision_msgs::msg::Detection2DArray>(
        //      topic, queue_size, std::bind(&Detect_Sub::callback, this, _1));

        detect_topic_=topic;
        if(is_trace)
            std::cout << " activate_detect() topic: "<< detect_topic_ << std::endl;

        subs_queue_.activate(node_,detect_topic_,queue_size);
    }

    bool is_trace=false;

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::string topic_;
    std::string detect_topic_;

    //std::shared_ptr<rclcpp::Subscription<vision_msgs::msg::Detection2DArray>> dtect_sub_;

    std::shared_ptr<vision_msgs::msg::Detection2DArray> dtec_msg_;

    //---------------
    // receve from /color/mobilenet_detections class
    //---------------
    Subs_Queue<vision_msgs::msg::Detection2DArray> subs_queue_;

    //---------------
    // void feedTopic() 
    // http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
    //---------------
    //void feedTopic(std::shared_ptr<RosMsg> msg){
    void feedTopic(std::shared_ptr<sensor_msgs::msg::Image> msg){

        cv::Mat rgb;

        int width = msg->width;
        int height = msg->height;
        //int step = msg->step;

        if(is_trace){
            std::cout << " Detect_Sub::feedTopic()"<<  std::endl;

            //std::cout << " width:"<< width <<  std::endl;
            //std::cout << " height:"<< height <<  std::endl;
            //std::cout << " step:"<< step <<  std::endl;
        }

        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        rgb = cv_ptr->image;

        //----
        // reffer from
        // /home/nishi/colcon_ws/src/depthai_ros_my/src/detect_com.cpp
        //   void Go_DtectPublish::feedImages(std::shared_ptr<dai::ADatatype> &Data)
        // 
        //-------
        std::shared_ptr<vision_msgs::msg::Detection2DArray> detectData;

        auto color = cv::Scalar(0, 255, 255);
        if (subs_queue_.msg_q_.size() > 0){
            detectData = subs_queue_.msg_q_.at(0);
            subs_queue_.msg_q_.pop_front();

            int d_size = detectData->detections.size();
            if(is_trace)
                std::cout << " detect_data->detections.size():" << detectData->detections.size() <<  std::endl;

            for(int i=0;i < d_size; i++ ){
                std::string class_name = detectData->detections[i].id;
                double score = detectData->detections[i].results[0].hypothesis.score;

                int xCenter = detectData->detections[i].bbox.center.position.x;
                int yCenter = detectData->detections[i].bbox.center.position.y;
                int xSize = detectData->detections[i].bbox.size_x;
                int ySize = detectData->detections[i].bbox.size_y;

                int x1 = xCenter - xSize/2;
                int x2 = x1 + xSize;
                int y1 = yCenter - ySize/2;
                int y2 = y1 + ySize;

                if(is_trace)
                    std::cout << " class_name:" << class_name <<  std::endl;

                std::stringstream confStr;
                confStr << std::fixed << std::setprecision(2) << score * 100;

                cv::putText(rgb, class_name, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
                cv::putText(rgb, confStr.str(), cv::Point(x1 + 10, y1 + 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
                cv::rectangle(rgb, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
            }
        }
        cv::imshow("rgb", rgb);
        int key = cv::waitKey(1);
    }

    //void callback(const std::shared_ptr<vision_msgs::msg::Detection2DArray> msg)
    //{
    //    //std::cout << "Subs_Com:: callback() I heard: " << std::endl;
    //    std::cout << " Detect_Sub::callback()"<<  std::endl;
    //    //feedTopic(msg);
    //}

};

}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("detect_subscriber");

    std::cout << "start detect_subscriber" << std::endl;

    std::string topic;
    std::string detect_topic;
    int queue_size;

    node->declare_parameter("topic", "color/image");
    node->declare_parameter("detect_topic", "color/mobilenet_detections");
    node->declare_parameter("queue_size", 10);

    node->get_parameter("topic", topic);
    node->get_parameter("detect_topic", detect_topic);
    node->get_parameter("queue_size", queue_size);

    //subs_com::Detect_Subs<sensor_msgs::msg::Image> detect;
    subs_com::Detect_Subs detect;
 
    //detect.is_trace=true;
    detect.activate_image(node,topic,queue_size);
    detect.activate_detect(detect_topic,queue_size);

    rclcpp::spin(node);
    
    return 0;
}

