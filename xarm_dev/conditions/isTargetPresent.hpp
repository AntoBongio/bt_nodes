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

class isTargetPresent : public BT::ConditionNode
{
public:
  using Bool = std_msgs::msg::Bool;

    isTargetPresent(const std::string& name, const BT::NodeConfiguration& config, std::string & ns, std::string topic_prefix_target_presence)
        : BT::ConditionNode(name, config)
    {
      
        node_ = rclcpp::Node::make_shared("is_target_present", ns);

        target_type_ = "";
        if (!getInput<std::string>("target", target_type_)) {
            // if I can't get this, there is something wrong with your BT.
            // For this reason throw an exception instead of returning FAILURE
            throw BT::RuntimeError("missing required input [target]");
        }

        // std::string target = target_type_.substr(0, target_type_.find("_"));

        // if(target == "aruco") {
          
        //   std::string target_aruco_id = "";
        //   target_aruco_id = target_type_;
        //   target_aruco_id = target_aruco_id.substr(target.length() + 1);

        // }
        // else{
        //     throw BT::RuntimeError("Unknown Target Type");
        // }

        target_presence_sub_ = node_->create_subscription<Bool>(topic_prefix_target_presence + target_type_, 1, 
          std::bind(&isTargetPresent::topic_callback, this, std::placeholders::_1));

        target_present_ = false;

        RCLCPP_INFO(node_->get_logger(), "isTargetPresent - init");
        
        exec_.add_node(node_);
        std::thread( [&] {this->exec_.spin();} ).detach();

    }

    static BT::PortsList providedPorts()
    {
        return{BT::InputPort<std::string>("target")};
    }

    virtual BT::NodeStatus tick() override
    {
      // RCLCPP_INFO(node_->get_logger(), "%s presence: %d", target_type_.c_str(), target_present_);
      return target_present_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }


    void topic_callback(const Bool::ConstSharedPtr msg)
    {
      target_present_ = msg->data;
    }


  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<Bool>::SharedPtr target_presence_sub_;
    rclcpp::executors::SingleThreadedExecutor exec_;

    std::string target_type_;

    bool target_present_;
};
