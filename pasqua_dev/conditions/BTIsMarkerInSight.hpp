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

class IsMarkerInSight : public BT::ConditionNode
{
public:
  using Bool = std_msgs::msg::Bool;

    IsMarkerInSight(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("is_marker_in_sight_bt");

        marker_id = 0;
        if (!getInput<int>("marker_id", marker_id)) {
            // if I can't get this, there is something wrong with your BT.
            // For this reason throw an exception instead of returning FAILURE
            throw BT::RuntimeError("missing required input [marker_id]");
        }

        this->marker_presence_sub = node_->create_subscription<Bool>("/target_tracking/camera_to_marker_presence/marker_" + std::to_string(marker_id), 1, 
              std::bind(&IsMarkerInSight::topic_callback, this, std::placeholders::_1));

        this->finish_wait = false;
        this->marker_in_sight = false;
        
        exec_.add_node(node_);
        std::thread( [&] {this->exec_.spin();} ).detach();

    }

    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<double>("wait_ms"), BT::InputPort<std::string>("marker_id")};
    }

    virtual BT::NodeStatus tick() override
    {
        int wait_ms = 0.0;
        if (!getInput<int>("wait_ms", wait_ms)) {
            // if I can't get this, there is something wrong with your BT.
            // For this reason throw an exception instead of returning FAILURE
            throw BT::RuntimeError("missing required input [wait_ms]");
        }


        if (wait_ms > 0)
        {
            std::chrono::milliseconds wait_ms_lit = std::chrono::milliseconds(wait_ms);
            timer_ = node_->create_wall_timer(wait_ms_lit, std::bind(&IsMarkerInSight::timer_callback, this));
        
            msg_received = false;
            while(not this->finish_wait and not msg_received){
              std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        return this->marker_in_sight ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    void timer_callback()
    {
      this->finish_wait = true;
      this->timer_->cancel();
    }

    void topic_callback(const Bool::ConstSharedPtr msg)
    {
      this->marker_in_sight = msg->data;
      msg_received = true;
    }


  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<Bool>::SharedPtr marker_presence_sub;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::executors::SingleThreadedExecutor exec_;

    int marker_id;

    bool marker_in_sight;
    bool finish_wait;
    bool msg_received;
};
