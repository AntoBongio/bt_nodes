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

#include "parking_interfaces/msg/parking_status.hpp"

using namespace std::chrono_literals;

class IsFineApproachFinished : public BT::ConditionNode
{
public:
  using ParkingStatus = parking_interfaces::msg::ParkingStatus;

    IsFineApproachFinished(const std::string& name, const BT::NodeConfiguration& config,  
          std::string ns, std::string parking_status_topic)
        : BT::ConditionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("is_fine_approach_finished_bt", ns);
        this->parking_status_sub = node_->create_subscription<ParkingStatus>(parking_status_topic, 1, 
              std::bind(&IsFineApproachFinished::topic_callback, this, std::placeholders::_1));

        this->finish_wait = false;
        this->msg_received = false;
        this->parking_status = -100;
        
        exec_.add_node(node_);
        std::thread( [&] {this->exec_.spin();} ).detach();

        RCLCPP_INFO(node_->get_logger(), "IsFineApproachFinished - init");
    }

    static BT::PortsList providedPorts()
    {
        return{};
    }

    virtual BT::NodeStatus tick() override
    {
        if(parking_status == 0)
          parking_finished = true;
        else
          parking_finished = false;

        return this->parking_finished ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    void topic_callback(const ParkingStatus::ConstSharedPtr msg)
    {
      this->parking_status = msg->status;
      msg_received = true;
    }


  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<ParkingStatus>::SharedPtr parking_status_sub;
    rclcpp::executors::SingleThreadedExecutor exec_;

    int parking_status;
    bool finish_wait;
    bool msg_received;
    bool parking_finished;
};
