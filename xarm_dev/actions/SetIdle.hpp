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

#include "std_srvs/srv/trigger.hpp"
#include "xarm_test_moveit_interfaces/action/follow_target.hpp"
#include "xarm_test_moveit_interfaces/action/approach_target.hpp"
#include "xarm_test_moveit_interfaces/action/pull_back.hpp"

using namespace std::chrono_literals;

class SetIdle : public BT::SyncActionNode
{
public:
  using Trigger = std_srvs::srv::Trigger;
  using FollowTarget = xarm_test_moveit_interfaces::action::FollowTarget;
  using ApproachTarget = xarm_test_moveit_interfaces::action::ApproachTarget;
  using PullBack = xarm_test_moveit_interfaces::action::PullBack;

  SetIdle(const std::string& name, const BT::NodeConfiguration& config, std::string & ns)
    : BT::SyncActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("set_idle", ns);
    
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

    client_pull_back_ = rclcpp_action::create_client<PullBack>(
      node_,
      "/pull_back");

    while(!client_pull_back_->wait_for_action_server()) {
      RCLCPP_INFO(node_->get_logger(), "Action server Pull Back not available after waiting");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    

    client_set_idle_ = node_->create_client<Trigger>("/set_idle");

    while(!client_set_idle_->wait_for_service(1s)) {
      RCLCPP_INFO(node_->get_logger(), "Client not available - waiting");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(node_->get_logger(), "SetIdle - init");

    exec_.add_node(node_);
    std::thread( [&] {this->exec_.spin();} ).detach();
  }

  static BT::PortsList providedPorts()
  {
    return{};
  }

  virtual BT::NodeStatus tick() override
  {

    RCLCPP_INFO(node_->get_logger(), "Set Idle BT.");

    service_finished_ = false;
    returned_value_ = false;

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

    //? Deactivate all pull back actions
    bool pull_back_end = false;
    auto result_pull_back = client_pull_back_->async_cancel_all_goals(
      [&](std::shared_ptr<action_msgs::srv::CancelGoal_Response> cancel){
        pull_back_end = true;
      });

    while(not pull_back_end) {
      std::this_thread::sleep_for(10ms);
    }

    //? SetIdle server request
    auto request = std::make_shared<Trigger::Request>();

    using ServiceResponseFuture =
      rclcpp::Client<Trigger>::SharedFutureWithRequest;
    auto response_received_callback =
      [&](ServiceResponseFuture future) {
        auto request_response_pair = future.get();
        service_finished_ = true;
        returned_value_ = request_response_pair.second->success;
      };

    auto result = client_set_idle_->async_send_request(request, std::move(response_received_callback));

    while(!service_finished_){
        std::this_thread::sleep_for(10ms);
    }
    
    // return returned_value_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    return BT::NodeStatus::SUCCESS;

  }




private:
  rclcpp::executors::SingleThreadedExecutor exec_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<Trigger>::SharedPtr  client_set_idle_;
  rclcpp_action::Client<FollowTarget>::SharedPtr client_follow_target_;
  rclcpp_action::Client<ApproachTarget>::SharedPtr client_approach_target_;
  rclcpp_action::Client<PullBack>::SharedPtr client_pull_back_;
  
  
  bool service_finished_;
  bool returned_value_;
};
