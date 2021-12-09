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

#include "xarm_test_moveit_interfaces/action/pull_back.hpp"

#include "xarm_test_moveit_interfaces/action/follow_target.hpp"
#include "xarm_test_moveit_interfaces/action/approach_target.hpp"


class PullBack : public BT::AsyncActionNode
{
public:
  using PullBackAction = xarm_test_moveit_interfaces::action::PullBack;
  using GoalHandlePullBack = rclcpp_action::ClientGoalHandle<PullBackAction>;

  using FollowTarget = xarm_test_moveit_interfaces::action::FollowTarget;
  using ApproachTarget = xarm_test_moveit_interfaces::action::ApproachTarget;

  PullBack(const std::string& name, const BT::NodeConfiguration& config, std::string & ns)
    : BT::AsyncActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("pull_back", ns);

    client_ptr_ = rclcpp_action::create_client<PullBackAction>(
      node_,
      "/pull_back");

    while(!client_ptr_->wait_for_action_server()) {
      RCLCPP_INFO(node_->get_logger(), "Action server pull back not available after waiting.");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    client_follow_target_ = rclcpp_action::create_client<FollowTarget>(
      node_,
      "/follow_target");

    while(!client_follow_target_->wait_for_action_server()) {
      RCLCPP_INFO(node_->get_logger(), "Action server Follow Target not available after waiting");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    client_approach_target_ = rclcpp_action::create_client<ApproachTarget>(
      node_,
      "/approach_target");

    while(!client_approach_target_->wait_for_action_server()) {
      RCLCPP_INFO(node_->get_logger(), "Action server Approach Target not available after waiting");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }


    send_goal_options_ = rclcpp_action::Client<PullBackAction>::SendGoalOptions();
    send_goal_options_.goal_response_callback =
      std::bind(&PullBack::goal_response_callback, this, std::placeholders::_1);
    send_goal_options_.feedback_callback =
      std::bind(&PullBack::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options_.result_callback =
      std::bind(&PullBack::result_callback, this, std::placeholders::_1);
      
    RCLCPP_INFO(node_->get_logger(), "Pull Back - init");

    exec_.add_node(node_);
    std::thread( [&] {exec_.spin();} ).detach();
  }

  static BT::PortsList providedPorts()
  {
    return{ };
  }

  virtual BT::NodeStatus tick() override
  {

    //? Deactivate all follow target actions
    bool follow_target_end = false;
    auto result_follow = client_follow_target_->async_cancel_all_goals(
      [&](std::shared_ptr<action_msgs::srv::CancelGoal_Response> cancel){
        follow_target_end = true;
      });

    while(not follow_target_end) {
      std::this_thread::sleep_for(10ms);
    }

    //? Deactivate all approach target actions
    bool approach_target_end = false;
    auto result_approach = client_approach_target_->async_cancel_all_goals(
      [&](std::shared_ptr<action_msgs::srv::CancelGoal_Response> cancel){
        approach_target_end = true;
      });

    while(not approach_target_end) {
      std::this_thread::sleep_for(10ms);
    }


    goal_msg = PullBackAction::Goal();

    goal_msg.pull_back = {0.08, 0.0};

    action_finished_ = false;
    
    client_ptr_->async_send_goal(goal_msg, send_goal_options_);

    while(not action_finished_){
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    return returned_value_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  void goal_response_callback(std::shared_future<GoalHandlePullBack::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_INFO(node_->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(GoalHandlePullBack::SharedPtr, const std::shared_ptr<const PullBackAction::Feedback>)
  {
    
  }

  void result_callback(const GoalHandlePullBack::WrappedResult & result)
  {
    switch (result.code) 
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_INFO(node_->get_logger(), "Goal was aborted");
        action_finished_ = true;
        returned_value_ = false;
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(node_->get_logger(), "Goal was canceled");
        action_finished_ = true;
        returned_value_ = false;
        return;
      default:
        RCLCPP_INFO(node_->get_logger(), "Unknown result code");
        return;
    }
    // std::stringstream ss;
    // ss << "Result received: " << result.result->sequence;
    // RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    
    action_finished_ = true;
    returned_value_ = true;
  }

  // This overloaded method is used to stop the execution of this node.
  void halt() override
  {
      RCLCPP_INFO(node_->get_logger(), "Pull Back - halt requested");

      //? Deactivate the action
      auto result_approach = client_ptr_->async_cancel_all_goals(
        [&](std::shared_ptr<action_msgs::srv::CancelGoal_Response> cancel){
          action_finished_ = true;
        });

      returned_value_ = false;
  }

private:
  rclcpp::executors::SingleThreadedExecutor exec_;

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<PullBackAction>::SharedPtr client_ptr_;
  rclcpp_action::Client<FollowTarget>::SharedPtr client_follow_target_;
  rclcpp_action::Client<ApproachTarget>::SharedPtr client_approach_target_;

  rclcpp_action::Client<PullBackAction>::SendGoalOptions send_goal_options_;
  PullBackAction::Goal goal_msg;

  std::string target_type_;
  bool action_finished_;
  bool returned_value_;
};
