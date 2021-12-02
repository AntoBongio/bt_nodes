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

class LocInitZero : public BT::SyncActionNode
{
public:
  using LocInit = localisation_interfaces::action::LocInit;
  using GoalHandleLocInit = rclcpp_action::ClientGoalHandle<LocInit>;

    LocInitZero(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("loc_zero_init_bt");
        this->client_ptr_ = rclcpp_action::create_client<LocInit>(
          node_,
          "/localisation_init_zero/start_action");

        while(!client_ptr_->wait_for_action_server()) {
          RCLCPP_INFO(node_->get_logger(), "Action server not available after waiting");
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        send_goal_options = rclcpp_action::Client<LocInit>::SendGoalOptions();
        send_goal_options.goal_response_callback =
          std::bind(&LocInitZero::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
          std::bind(&LocInitZero::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
          std::bind(&LocInitZero::result_callback, this, std::placeholders::_1);

        RCLCPP_INFO(node_->get_logger(), "LocInitZero - init");
    }

    static BT::PortsList providedPorts()
    {
        return{};
    }

    virtual BT::NodeStatus tick() override
    {
        

        _halt_requested.store(false);

        goal_msg = LocInit::Goal();

        client_ptr_->async_send_goal(goal_msg, send_goal_options);

        while(!this->snooping_finished){
            rclcpp::spin_some(node_);
        }
      
        RCLCPP_INFO(node_->get_logger(), "LocInitZero - FINISHED");
        return _halt_requested or not this->returned_value ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
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
    rclcpp_action::Client<LocInit>::SharedPtr client_ptr_;

    rclcpp_action::Client<LocInitZero::LocInit>::SendGoalOptions send_goal_options;
    localisation_interfaces::action::LocInit::Goal goal_msg;
    
    std::atomic_bool _halt_requested;
    bool snooping_finished;
    bool returned_value;
};
