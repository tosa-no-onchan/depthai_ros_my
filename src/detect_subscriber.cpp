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
*   $ ros2 run depthai_ros_my detect_subscriber [--ros-args -p topic:=color/video/image -p detect_topic:=color/tiny_yolo_detections]
*
* reffer from
*   http://rpgincpp.cocolog-nifty.com/blog/2006/02/post_0123.html
*   https://cpprefjp.github.io/lang/cpp23/class_template_argument_deduction_from_inherited.html
*   https://qiita.com/srs/items/9ab7456e4a3a4c5cf645
*
*/

//#include "depthai/depthai.hpp"

#include "depthai_ros_my/subs_com.hpp"
#include <opencv2/opencv.hpp>
//#include <cv_bridge/cv_bridge.h>
// changed by nishi 2025.3.3
#include <cv_bridge/cv_bridge.hpp>

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
        if(is_trace_)
            std::cout << " activate_image() topic: "<< topic_ << std::endl;

        //Subs_Com<RosMsg>::init(node_,topic_, queue_size);   // OK
        Subs_Com<sensor_msgs::msg::Image>::init(node_,topic_, queue_size);   // OK
        //this->init(node_,topic_,queue_size);      // OK
    }

    void activate_detect(std::string topic, int queue_size=10,bool normalized=true){
        //dtect_sub_ = node_->create_subscription<vision_msgs::msg::Detection2DArray>(
        //      topic, queue_size, std::bind(&Detect_Sub::callback, this, _1));

        detect_topic_=topic;
        normalized_=normalized;
        if(is_trace_)
            std::cout << " activate_detect() topic: "<< detect_topic_ << std::endl;

        subs_queue_.activate(node_,detect_topic_,queue_size);
    }

    void set_as_rate(double as_rate=1.0){
        as_rate_ = as_rate;
        is_as_rate_=true;
    }

    bool is_trace_=false;
    bool normalized_=true;
    bool is_as_rate_=false;

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::string topic_;
    std::string detect_topic_;

    double as_rate_;     // h/w rate

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
        int off_x=0;
        int off_y=0;

        //int tmp_w,tmp_h;
        int as_height,as_width;

        // aspect rate 調整します。
        if(normalized_==true && is_as_rate_ == true){
            // aspect rate に従った as_height を求める
            as_height = (int)((double)width * as_rate_);
            // 入力画像 height が、小さい
            if(height < as_height){
                // as_width を基準にして、処理する。
                as_width = (int)((double)height / as_rate_);
                // x軸の起点を調整。
                off_x = (width - as_width)/2;
                // 入力画像 width を、aspect width にする。
                width = as_width;
            }
            // 入力画像 height が、大きい
            else if(height > as_height){
                // as_height を基準にして、処理する。
                // y軸の起点を調整。
                off_y = (height - as_height)/2;
                // 入力画像 height を、aspect height にする。
                height= as_height;
            }
        }

        if(is_trace_){
            std::cout << " Detect_Sub::feedTopic()"<<  std::endl;

            std::cout << " width:"<< width <<  std::endl;
            std::cout << " height:"<< height <<  std::endl;
            //std::cout << " step:"<< step <<  std::endl;
        }

        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

        if(is_trace_){
            std::cout << " cv_ptr->encoding:" << cv_ptr->encoding << std::endl;
        }
        if(cv_ptr->encoding=="rgb8"){
            cv::cvtColor(cv_ptr->image,rgb,cv::COLOR_RGB2BGR);
        }
        else{
            rgb = cv_ptr->image;
        }

        //if(is_trace_){
        //    std::cout << " rgb.cols:"<< rgb.cols <<  std::endl;
        //    std::cout << " rgb.rows:"<< rgb.rows <<  std::endl;
        //}

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
            if(is_trace_)
                std::cout << " detect_data->detections.size():" << detectData->detections.size() <<  std::endl;

            for(int i=0;i < d_size; i++ ){
                std::string class_name = detectData->detections[i].id;
                double score = detectData->detections[i].results[0].hypothesis.score;

                double xCenter = detectData->detections[i].bbox.center.position.x;
                double yCenter = detectData->detections[i].bbox.center.position.y;
                double xSize = detectData->detections[i].bbox.size_x;
                double ySize = detectData->detections[i].bbox.size_y;

                int x1,x2,y1,y2;
                if(normalized_==true){
                    x1 = ((xCenter - xSize*0.5) * width) + off_x;
                    x2 = ((xCenter + xSize*0.5) * width) + off_x;
                    y1 = ((yCenter - ySize*0.5) * height) + off_y;
                    y2 = ((yCenter + ySize*0.5) * height) + off_y;
                }
                else{
                    x1 = xCenter - xSize/2;
                    x2 = x1 + xSize;
                    y1 = yCenter - ySize/2;
                    y2 = y1 + ySize;
                }

                if(is_trace_){
                    std::cout << " class_name:" << class_name <<  std::endl;

                    std::cout << " x1:"<< x1 <<" x2:"<< x2 << std::endl;
                    std::cout << " y1:"<< y1 <<" y2:"<< y2 << std::endl;
                }


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

    //node->declare_parameter("topic", "color/image");
    node->declare_parameter("topic", "color/video/image");
    //node->declare_parameter("detect_topic", "color/mobilenet_detections");
    node->declare_parameter("detect_topic", "color/tiny_yolo_detections");
    node->declare_parameter("queue_size", 10);

    node->get_parameter("topic", topic);
    node->get_parameter("detect_topic", detect_topic);
    node->get_parameter("queue_size", queue_size);

    //subs_com::Detect_Subs<sensor_msgs::msg::Image> detect;
    subs_com::Detect_Subs detect;
    detect.set_as_rate();
 
    //detect.is_trace=true;
    detect.activate_image(node,topic,queue_size);
    detect.activate_detect(detect_topic,queue_size);

    rclcpp::spin(node);
    
    return 0;
}

