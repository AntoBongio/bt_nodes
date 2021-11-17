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

#include "behaviortree_cpp_v3/action_node.h"

#include "marker_snooping_interfaces/action/snooping.hpp"

class SnoopMarker : public BT::SyncActionNode
{
public:
  using Snooping = marker_snooping_interfaces::action::Snooping;
  using GoalHandleSnooping = rclcpp_action::ClientGoalHandle<Snooping>;

    SnoopMarker(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("snoop_bt");
        this->client_ptr_ = rclcpp_action::create_client<Snooping>(
          node_,
          "/marker_snooping/start_action");

        while(!client_ptr_->wait_for_action_server()) {
          RCLCPP_INFO(node_->get_logger(), "Action server snoop not available after waiting");
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        send_goal_options = rclcpp_action::Client<Snooping>::SendGoalOptions();
        send_goal_options.goal_response_callback =
          std::bind(&SnoopMarker::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
          std::bind(&SnoopMarker::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
          std::bind(&SnoopMarker::result_callback, this, std::placeholders::_1);
        
        RCLCPP_INFO(node_->get_logger(), "SnoopMarker - init");
    }

    static BT::PortsList providedPorts()
    {
        return{ };
    }

    virtual BT::NodeStatus tick() override
    {
        goal_msg = Snooping::Goal();

        client_ptr_->async_send_goal(goal_msg, send_goal_options);

        while(!this->snooping_finished){
            rclcpp::spin_some(node_);
        }
        
        RCLCPP_INFO(node_->get_logger(), "SnoopMarker - FINISHED");
        return this->returned_value ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    void goal_response_callback(std::shared_future<GoalHandleSnooping::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
          RCLCPP_INFO(node_->get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(GoalHandleSnooping::SharedPtr,
        const std::shared_ptr<const Snooping::Feedback> feedback)
    {
        // std::stringstream ss;
        // ss << "percentage_of_completing: " << feedback->percentage_of_completing;
        // RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    }

    void result_callback(const GoalHandleSnooping::WrappedResult & result)
    {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_INFO(node_->get_logger(), "Goal was aborted");
            snooping_finished = true;
            returned_value = false;
            return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(node_->get_logger(), "Goal was canceled");
            snooping_finished = true;
            returned_value = false;
            return;
          default:
            RCLCPP_INFO(node_->get_logger(), "Unknown result code");
            return;
        }
        std::stringstream ss;
        ss << "Result received: " << result.result->ret;
        snooping_finished = true;
        returned_value = result.result->ret;
        RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    }

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<Snooping>::SharedPtr client_ptr_;

    rclcpp_action::Client<SnoopMarker::Snooping>::SendGoalOptions send_goal_options;
    marker_snooping_interfaces::action::Snooping::Goal goal_msg;

    bool snooping_finished;
    bool returned_value;
};
