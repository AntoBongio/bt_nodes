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

#include "localisation_interfaces/action/loc_init.hpp"

class LocInitMarker : public BT::AsyncActionNode
{
public:
  using LocInit = localisation_interfaces::action::LocInit;
  using GoalHandleLocInit = rclcpp_action::ClientGoalHandle<LocInit>;

    LocInitMarker(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("loc_marker_init_bt");
        this->client_ptr_ = rclcpp_action::create_client<LocInit>(
          node_,
          "/localisation_init_with_marker/start_action");

        while(!client_ptr_->wait_for_action_server()) {
          RCLCPP_INFO(node_->get_logger(), "Action server not available after waiting");
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        send_goal_options = rclcpp_action::Client<LocInit>::SendGoalOptions();
        send_goal_options.goal_response_callback =
          std::bind(&LocInitMarker::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
          std::bind(&LocInitMarker::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
          std::bind(&LocInitMarker::result_callback, this, std::placeholders::_1);


        RCLCPP_INFO(node_->get_logger(), "LocInitMarker - init");
    }

    static BT::PortsList providedPorts()
    {
        return{};
    }

    virtual BT::NodeStatus tick() override
    {
         goal_msg = LocInit::Goal();
         localisation_inited= false;

        client_ptr_->async_send_goal(goal_msg, send_goal_options);

        while(!this->localisation_inited){
            rclcpp::spin_some(node_);
        }
        
        RCLCPP_INFO(node_->get_logger(), "MarkerLocInit - FINISHED");
        return this->returned_value ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    void goal_response_callback(std::shared_future<GoalHandleLocInit::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
          RCLCPP_INFO(node_->get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(GoalHandleLocInit::SharedPtr,
        const std::shared_ptr<const LocInit::Feedback> feedback)
    {
        
    }

    void result_callback(const GoalHandleLocInit::WrappedResult & result)
    {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_INFO(node_->get_logger(), "Goal was aborted");
            localisation_inited = true;
            returned_value = false;
            return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(node_->get_logger(), "Goal was canceled");
            localisation_inited = true;
            returned_value = false;
            return;
          default:
            RCLCPP_INFO(node_->get_logger(), "Unknown result code");
            return;
        }
        std::stringstream ss;
        ss << "Result received: " << result.result->ret;
        localisation_inited = true;
        returned_value = result.result->ret;
        RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    }

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<LocInit>::SharedPtr client_ptr_;

    rclcpp_action::Client<LocInitMarker::LocInit>::SendGoalOptions send_goal_options;
    localisation_interfaces::action::LocInit::Goal goal_msg;

    bool localisation_inited;
    bool returned_value;
};
