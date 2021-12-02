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

#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::chrono_literals;

class CheckTargetDistance : public BT::ConditionNode
{
public:
    using TransformStamped = geometry_msgs::msg::TransformStamped;

    CheckTargetDistance(const std::string& name, const BT::NodeConfiguration& config, std::string & ns, std::string topic_prefix_marker_pose)
        : BT::ConditionNode(name, config)
    {
      
        node_ = rclcpp::Node::make_shared("check_xy_distance", ns);

        target_ = "";
        if (!getInput<std::string>("target", target_)) {
            // if I can't get this, there is something wrong with your BT.
            // For this reason throw an exception instead of returning FAILURE
            throw BT::RuntimeError("missing required input [x_limit]");
        }

        if (!getInput<double>("x_limit", x_limit_)) {
            // if I can't get this, there is something wrong with your BT.
            // For this reason throw an exception instead of returning FAILURE
            throw BT::RuntimeError("missing required input [x_limit]");
        }

        if (!getInput<double>("y_limit", y_limit_)) {
            // if I can't get this, there is something wrong with your BT.
            // For this reason throw an exception instead of returning FAILURE
            throw BT::RuntimeError("missing required input [y_limit]");
        }

        in_range_ = false;
        position_target_ = {0.0, 0.0, 0.0};

        RCLCPP_INFO(node_->get_logger(), "TOPIC: %s", topic_prefix_marker_pose.c_str());
        RCLCPP_INFO(node_->get_logger(), "TOPIC: %s", target_.c_str());
        RCLCPP_INFO(node_->get_logger(), "TOPIC: %s", (topic_prefix_marker_pose + target_).c_str());

        sub_target_position_ = node_->create_subscription<TransformStamped>(topic_prefix_marker_pose + target_, 1, 
          std::bind(&CheckTargetDistance::topic_callback, this, std::placeholders::_1));

        exec_.add_node(node_);
        std::thread( [&] {this->exec_.spin();} ).detach();

    }

    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("target"), BT::InputPort<double>("x_limit"), BT::InputPort<double>("y_limit") };
    }

    virtual BT::NodeStatus tick() override
    {
        if(std::abs(position_target_[0]) <= x_limit_ and std::abs(position_target_[1]) <= y_limit_ ) 
        {
            in_range_ = true;
        }
        else {
            in_range_ = false;
        }

        return in_range_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }


    void topic_callback(const TransformStamped::ConstSharedPtr msg)
    {   
        position_target_[0] = msg->transform.translation.x;
        position_target_[1] = msg->transform.translation.y;
        position_target_[2] = msg->transform.translation.z;
    }


  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<TransformStamped>::SharedPtr sub_target_position_;
    rclcpp::executors::SingleThreadedExecutor exec_;

    std::string target_;
    double x_limit_;
    double y_limit_;
    std::vector<double> position_target_;

    bool in_range_;
};
