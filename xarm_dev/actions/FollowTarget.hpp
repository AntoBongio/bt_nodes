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

#include "xarm_test_moveit_interfaces/action/follow_target.hpp"

#include "xarm_test_moveit_interfaces/action/approach_target.hpp"
#include "xarm_test_moveit_interfaces/action/pull_back.hpp"

class FollowTarget : public BT::AsyncActionNode
{
public:
  using FollowTargetAction = xarm_test_moveit_interfaces::action::FollowTarget;
  using GoalHandleFollowTarget = rclcpp_action::ClientGoalHandle<FollowTargetAction>;

  using ApproachTarget = xarm_test_moveit_interfaces::action::ApproachTarget;
  using PullBack = xarm_test_moveit_interfaces::action::PullBack;

  FollowTarget(const std::string& name, const BT::NodeConfiguration& config, std::string & ns)
    : BT::AsyncActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("follow_target", ns);

    target_type_ = "";
    if (!getInput<std::string>("target", target_type_)) {
      // if I can't get this, there is something wrong with your BT.
      // For this reason throw an exception instead of returning FAILURE
      throw BT::RuntimeError("missing required input [target]");
    }

    client_ptr_ = rclcpp_action::create_client<FollowTargetAction>(
      node_,
      "/follow_target");

    while(!client_ptr_->wait_for_action_server()) {
      RCLCPP_INFO(node_->get_logger(), "Action server follow target not available after waiting.");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    client_approach_target_ = rclcpp_action::create_client<ApproachTarget>(
      node_,
      "/approach_target");

    while(!client_approach_target_->wait_for_action_server()) {
      RCLCPP_INFO(node_->get_logger(), "Action server Approach Target not available after waiting");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    client_pull_back_ = rclcpp_action::create_client<PullBack>(
      node_,
      "/pull_back");

    while(!client_pull_back_->wait_for_action_server()) {
      RCLCPP_INFO(node_->get_logger(), "Action server Pull Back not available after waiting");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    send_goal_options_ = rclcpp_action::Client<FollowTargetAction>::SendGoalOptions();
    send_goal_options_.goal_response_callback =
      std::bind(&FollowTarget::goal_response_callback, this, std::placeholders::_1);
    send_goal_options_.feedback_callback =
      std::bind(&FollowTarget::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options_.result_callback =
      std::bind(&FollowTarget::result_callback, this, std::placeholders::_1);
      
    RCLCPP_INFO(node_->get_logger(), "FollowTarget - init");

    exec_.add_node(node_);
    std::thread( [&] {exec_.spin();} ).detach();
  }

  static BT::PortsList providedPorts()
  {
    return{ BT::InputPort<std::string>("target") };
  }

  virtual BT::NodeStatus tick() override
  {
    //? Deactivate all approach target actions
    bool approach_target_end = false;
    auto result_approach = client_approach_target_->async_cancel_all_goals(
      [&](std::shared_ptr<action_msgs::srv::CancelGoal_Response> cancel){
        approach_target_end = true;
      });

    while(not approach_target_end) {
      std::this_thread::sleep_for(10ms);
    }

    //? Deactivate all pull back actions
    bool pull_back_end = false;
    auto result_pull_back = client_pull_back_->async_cancel_all_goals(
      [&](std::shared_ptr<action_msgs::srv::CancelGoal_Response> cancel){
        pull_back_end = true;
      });

    while(not pull_back_end) {
      std::this_thread::sleep_for(10ms);
    }

    
    goal_msg = FollowTargetAction::Goal();

    goal_msg.target.data = target_type_;

    action_finished_ = false;
    
    client_ptr_->async_send_goal(goal_msg, send_goal_options_);

    while(not action_finished_){
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    return returned_value_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  void goal_response_callback(std::shared_future<GoalHandleFollowTarget::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_INFO(node_->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(GoalHandleFollowTarget::SharedPtr, const std::shared_ptr<const FollowTargetAction::Feedback>)
  {
    
  }

  void result_callback(const GoalHandleFollowTarget::WrappedResult & result)
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
      RCLCPP_INFO(node_->get_logger(), "FollowTarget - halt requested");

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
  rclcpp_action::Client<FollowTargetAction>::SharedPtr client_ptr_;
  rclcpp_action::Client<ApproachTarget>::SharedPtr client_approach_target_;
  rclcpp_action::Client<PullBack>::SharedPtr client_pull_back_;
  

  rclcpp_action::Client<FollowTargetAction>::SendGoalOptions send_goal_options_;
  FollowTargetAction::Goal goal_msg;

  std::string target_type_;
  bool action_finished_;
  bool returned_value_;
};
