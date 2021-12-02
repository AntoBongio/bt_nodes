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

class ReadTopicMessageExample : public BT::ConditionNode
{
public:
  using Bool = std_msgs::msg::Bool;

    ReadTopicMessageExample(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("read_topic_message_example_bt");
        this->sub_ = node_->create_subscription<Bool>("/topic_name", 1, 
              std::bind(&ReadTopicMessageExample::topic_callback, this, std::placeholders::_1));

        this->finish_waiting = false;
        this->message_from_topic = false;
        
        exec_.add_node(node_);
        std::thread( [&] {this->exec_.spin();} ).detach();

        RCLCPP_INFO(node_->get_logger(), "ReadTopicMessageExample - init");
    }

    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<double>("wait_ms")};
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
            msg_received = false;
            returned_value = false;

            std::chrono::milliseconds wait_ms_lit = std::chrono::milliseconds(wait_ms);
            timer_ = node_->create_wall_timer(wait_ms_lit, std::bind(&ReadTopicMessageExample::timer_callback, this));
        
            msg_received = false;
            while(not this->finish_waiting and not msg_received){
              std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        else{
            returned_value = true;
        }

        return this->returned_value ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    void timer_callback()
    {
      this->finish_waiting = true;
      this->timer_->cancel();
    }

    void topic_callback(const Bool::ConstSharedPtr msg)
    {
      this->message_from_topic = msg->data;
      msg_received = true;
      returned_value = true;
    }


  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<Bool>::SharedPtr sub_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::executors::SingleThreadedExecutor exec_;

    bool message_from_topic;

    bool returned_value;
    bool finish_waiting;
    bool msg_received;
};
