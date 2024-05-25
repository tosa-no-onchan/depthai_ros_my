/*
* detect_subscriber.cpp
*
* depthai_ros_my/include/depthai_ros_my/subs_com.hpp
*
*/

#pragma once


#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include <cstdio>
#include <iostream>
#include <tuple>

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

#include <deque>


namespace subs_com{

using std::placeholders::_1;


//-------------------
// class Subs_Com
//-------------------
// std_msgs::msg::String
// sensor_msgs::msg::Image
// vision_msgs::msg::Detection2DArray
template <class T> class Subs_Com{
public:
    std::shared_ptr<rclcpp::Node> node_;

    //#define TEST_22
    #if defined(TEST_22)
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        //rclcpp::Subscription<RosMsg>::SharedPtr subscription_;
    #else
        std::shared_ptr<rclcpp::Subscription<T>> subscription_;
    #endif

    std::string topic_;
    int queue_size_;

    Subs_Com(){}
    void init(std::shared_ptr<rclcpp::Node> node,std::string topic, int queue_size=10){
        node_=node;
        topic_=topic;
        queue_size_=queue_size;

        #if defined(TEST_22)
            subscription_ = node_->create_subscription<std_msgs::msg::String>(
            "topic", 10, std::bind(&Subs_Com::topic_callback, this, _1));
        #else
            subscription_ = node_->create_subscription<T>(
                topic_, queue_size_, std::bind(&Subs_Com::callback, this, _1));
        #endif

    }

    virtual void feedTopic(std::shared_ptr<T> msg){}

private:
    #if defined(TEST_22)
        void callback(const std_msgs::msg::String::SharedPtr msg) const
        {
            //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
            std::cout << "Subs_Com:: callback() I heard: " << std::endl;
        }
    #else
        //void callback(const std_msgs::msg::String::SharedPtr msg) const
        void callback(const std::shared_ptr<T> msg)
        {
            //std::cout << "Subs_Com:: callback() I heard: " << std::endl;
            feedTopic(msg);
        }
    #endif
};

//-------------------
// class Subs_Queue
//-------------------
template <class RosMsg> class Subs_Queue:public Subs_Com<RosMsg>{
public:
    Subs_Queue(){}
    void activate(std::shared_ptr<rclcpp::Node> node, std::string topic, int queue_size){

        if(is_trace_)
            std::cout << " Subs_Queue::activate()"<<  std::endl;

        node_=node;
        topic_=topic;
        queue_size_=queue_size;
        Subs_Com<RosMsg>::init(node_,topic_,queue_size_);   // OK
        //this->init(node_,topic_,queue_size);      // OK
    }

    void feedTopic(std::shared_ptr<RosMsg> msg){
        //std::cout << "I heard: "<< msg->data.c_str() << std::endl;
        if(is_trace_)
            std::cout << " Subs_Queue::feedTopic()"<<  std::endl;
        if(msg_q_.size() < queue_size_){
            msg_q_.push_back(msg);
        }
        else{
            if(is_trace_)
                std::cout << " Subs_Queue:: msg_q_ is over flow!!"<<  std::endl;
        }
    }

    std::deque<std::shared_ptr<RosMsg>> msg_q_;

    void set_trace(bool trace=true){
        is_trace_=trace;
    }
    bool is_trace_=false;
private:
    std::shared_ptr<rclcpp::Node> node_;
    std::string topic_;
    int queue_size_;

};

}