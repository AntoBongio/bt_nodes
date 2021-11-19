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

    IsFineApproachFinished(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("is_fine_approach_finished_bt");
        this->parking_status_sub = node_->create_subscription<ParkingStatus>("/parking/status", 1, 
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
        return{ BT::InputPort<double>("wait_ms")};
    }

    virtual BT::NodeStatus tick() override
    {
        if (wait_ms > 0)
        {
          timer_ = node_->create_wall_timer(wait_ms_lit, std::bind(&IsFineApproachFinished::timer_callback, this));
        
          msg_received = false;
          while(not this->finish_wait and not msg_received){
              std::this_thread::sleep_for(std::chrono::milliseconds(10));
          }
        }

        if(parking_status == 0)
          parking_finished = true;
        else
          parking_finished = false;

        return this->parking_finished ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    void timer_callback()
    {
      this->finish_wait = true;
      this->timer_->cancel();
    }

    void topic_callback(const ParkingStatus::ConstSharedPtr msg)
    {
      this->parking_status = msg->status;
      msg_received = true;
    }


  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<ParkingStatus>::SharedPtr parking_status_sub;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::executors::SingleThreadedExecutor exec_;

    int wait_ms;
    std::chrono::milliseconds wait_ms_lit;

    int parking_status;
    bool finish_wait;
    bool msg_received;
    bool parking_finished;
};
