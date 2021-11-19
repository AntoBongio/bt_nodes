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

#include "controller_interfaces/action/fine_approach.hpp"

class FineApproachManeuver : public BT::AsyncActionNode
{
public:
  using FineApproach = controller_interfaces::action::FineApproach;
  using GoalHandleFineApproach = rclcpp_action::ClientGoalHandle<FineApproach>;

    FineApproachManeuver(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("fine_approach_maneuver_bt");
        this->client_ptr_ = rclcpp_action::create_client<FineApproach>(
          node_,
          "/pasqua_controller/fine_approach");

        while(!client_ptr_->wait_for_action_server()) {
          RCLCPP_INFO(node_->get_logger(), "Action server fine_approach not available after waiting");
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        exec_.add_node(node_);
        std::thread( [&] {this->exec_.spin();} ).detach();

        send_goal_options = rclcpp_action::Client<FineApproach>::SendGoalOptions();
        send_goal_options.goal_response_callback =
          std::bind(&FineApproachManeuver::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
          std::bind(&FineApproachManeuver::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
          std::bind(&FineApproachManeuver::result_callback, this, std::placeholders::_1);
          
        RCLCPP_INFO(node_->get_logger(), "FineApproachManeuver - init");
    }

    static BT::PortsList providedPorts()
    {
        return{ };
    }

    virtual BT::NodeStatus tick() override
    {
        goal_msg = FineApproach::Goal();

        _halt_requested.store(false);
        fine_approach_finished = false;
        
        client_ptr_->async_send_goal(goal_msg, send_goal_options);

        while(not this->fine_approach_finished){
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        RCLCPP_INFO(node_->get_logger(), "FineApproachManeuver - FINISHED");
        return this->returned_value and not _halt_requested ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    void goal_response_callback(std::shared_future<GoalHandleFineApproach::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
          RCLCPP_INFO(node_->get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(GoalHandleFineApproach::SharedPtr,
        const std::shared_ptr<const FineApproach::Feedback>)
    {
        
    }

    void result_callback(const GoalHandleFineApproach::WrappedResult & result)
    {
      if (not _halt_requested)
      {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_INFO(node_->get_logger(), "Goal was aborted");
            fine_approach_finished = true;
            returned_value = false;
            return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(node_->get_logger(), "Goal was canceled");
            fine_approach_finished = true;
            returned_value = false;
            return;
          default:
            RCLCPP_INFO(node_->get_logger(), "Unknown result code");
            return;
        }
        // std::stringstream ss;
        // ss << "Result received: " << result.result->ret;
        // RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
        
        fine_approach_finished = true;
        returned_value = result.result->ret;
      }
    }

    // This overloaded method is used to stop the execution of this node.
    void halt() override
    {
        RCLCPP_INFO(node_->get_logger(), "FineApproachManeuver - halt requested");
        fine_approach_finished = true;
        returned_value = false;
    }

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<FineApproach>::SharedPtr client_ptr_;
    rclcpp::executors::SingleThreadedExecutor exec_;

    rclcpp_action::Client<FineApproachManeuver::FineApproach>::SendGoalOptions send_goal_options;
    controller_interfaces::action::FineApproach::Goal goal_msg;

    std::atomic_bool _halt_requested;
    bool fine_approach_finished;
    bool returned_value;
};
