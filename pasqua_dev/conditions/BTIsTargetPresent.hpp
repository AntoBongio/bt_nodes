#pragma once

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "behaviortree_cpp_v3/condition_node.h"

#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class IsTargetPresent : public BT::ConditionNode
{
public:
  using Bool = std_msgs::msg::Bool;

    IsTargetPresent(const std::string& name, const BT::NodeConfiguration& config, std::string target_presence_topic)
        : BT::ConditionNode(name, config)
    {
      
        node_ = rclcpp::Node::make_shared("is_target_present_bt");

        target_type_ = "";
        if (!getInput<std::string>("target_type", target_type_)) {
            // if I can't get this, there is something wrong with your BT.
            // For this reason throw an exception instead of returning FAILURE
            throw BT::RuntimeError("missing required input [target_type]");
        }

        std::string target_str_ = target_type_.substr(0, target_type_.find("_"));

        if(target_str_ == "aruco") {
          
          std::string target_aruco_id_ = "";
          target_aruco_id_ = target_type_;
          target_aruco_id_ = target_aruco_id_.substr(target_str_.length() + 1);


          this->target_presence_sub_ = node_->create_subscription<Bool>(target_presence_topic + target_aruco_id_, 1, 
              std::bind(&IsTargetPresent::topic_callback, this, std::placeholders::_1));

        }
        else{
            throw BT::RuntimeError("Unknown Target Type");
        }

        this->finish_waiting_ = false;
        this->target_present_ = false;
        
        exec_.add_node(node_);
        std::thread( [&] {this->exec_.spin();} ).detach();

    }

    static BT::PortsList providedPorts()
    {
        return{BT::InputPort<std::string>("target_type")};
    }

    virtual BT::NodeStatus tick() override
    {
        return this->target_present_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }


    void topic_callback(const Bool::ConstSharedPtr msg)
    {
      this->target_present_ = msg->data;
      msg_received_ = true;
    }


  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<Bool>::SharedPtr target_presence_sub_;
    rclcpp::executors::SingleThreadedExecutor exec_;

    std::string target_type_;

    bool target_present_;
    bool finish_waiting_;
    bool msg_received_;
};
