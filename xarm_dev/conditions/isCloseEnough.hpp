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

#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace std::chrono_literals;

class isCloseEnough : public BT::ConditionNode
{
public:
    using PoseStamped = geometry_msgs::msg::PoseStamped;

    isCloseEnough(const std::string& name, const BT::NodeConfiguration& config, std::string & ns, std::string link_base_x_eef_topic, 
        std::vector<double> & x_limits, std::vector<double> & y_limits, std::vector<double> & z_limits)
        : BT::ConditionNode(name, config)
        , x_limits_{std::make_pair(x_limits[0], x_limits[1])}
        , y_limits_{std::make_pair(y_limits[0], y_limits[1])}
        , z_limits_{std::make_pair(z_limits[0], z_limits[1])}
    {
      
        node_ = rclcpp::Node::make_shared("is_target_present", ns);

        in_range_ = false;
        link_base__x_ref_ = {0.0, 0.0, 0.0};
        
        sub_link_base_pose_ref_ = node_->create_subscription<PoseStamped>(
            link_base_x_eef_topic, 1, std::bind(&isCloseEnough::topic_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(node_->get_logger(), "From isCloneEnough, x_limist: [%f, %f]", x_limits[0], x_limits[1]);

        exec_.add_node(node_);
        std::thread( [&] {this->exec_.spin();} ).detach();

    }

    static BT::PortsList providedPorts()
    {
        return{};
    }

    virtual BT::NodeStatus tick() override
    {
        if(link_base__x_ref_[0] >= x_limits_.first and link_base__x_ref_[0] <= x_limits_.second and
            link_base__x_ref_[1] >= y_limits_.first and link_base__x_ref_[1] <= y_limits_.second and
            link_base__x_ref_[2] >= z_limits_.first and link_base__x_ref_[2] <= z_limits_.second ) 
        {
            in_range_ = true;
        }
        else {
            in_range_ = false;
        }

        return in_range_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }


    void topic_callback(const PoseStamped::ConstSharedPtr msg)
    {
        link_base__x_ref_[0] = msg->pose.position.x;
        link_base__x_ref_[1] = msg->pose.position.y;
        link_base__x_ref_[2] = msg->pose.position.z;
    }


  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<PoseStamped>::SharedPtr sub_link_base_pose_ref_;
    rclcpp::executors::SingleThreadedExecutor exec_;

    std::pair<double, double> x_limits_;
    std::pair<double, double> y_limits_;
    std::pair<double, double> z_limits_;
    std::vector<double> link_base__x_ref_;

    bool in_range_;
};
