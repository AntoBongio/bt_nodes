#pragma once

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "behaviortree_cpp_v3/action_node.h"

#include "xarm_test_moveit_interfaces/srv/change_gripper_state.hpp"

using namespace std::chrono_literals;

#define STATE_MAX 0.75 // Close
#define STATE_MIN 0 // Open

class ChangeGripperState : public BT::SyncActionNode
{
public:
  using ChangeGripperStateService = xarm_test_moveit_interfaces::srv::ChangeGripperState;

  ChangeGripperState(const std::string& name, const BT::NodeConfiguration& config, std::string & ns)
      : BT::SyncActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("change_gripper_state", ns);
    this->client_ptr_ = node_->create_client<ChangeGripperStateService>(
      "/change_gripper_state");

    while(!client_ptr_->wait_for_service(1s)) {
      RCLCPP_INFO(node_->get_logger(), "Client not available - waiting");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    state_ = "";
    if (!getInput<std::string>("state", state_)) {
    // if I can't get this, there is something wrong with your BT.
    // For this reason throw an exception instead of returning FAILURE
    throw BT::RuntimeError("missing required input [target_type]");
    }
    if(state_ == "open")
        state_numeric_ = 0.0;
    else if(state_ == "close")
        state_numeric_ = 0.75;
    else 
        state_numeric_ = std::stof(state_);

    RCLCPP_INFO(node_->get_logger(), "ChangeGripperState - init");

    exec_.add_node(node_);
    std::thread( [&] {exec_.spin();} ).detach();
  }

  static BT::PortsList providedPorts()
  {
    return{ BT::InputPort<std::string>("state") };
  }

  virtual BT::NodeStatus tick() override
  {
    service_finished_ = false;
    returned_value_ = false;

    auto request = std::make_shared<ChangeGripperStateService::Request>();

    request->closure = state_numeric_;

    using ServiceResponseFuture =
      rclcpp::Client<ChangeGripperStateService>::SharedFutureWithRequest;
    auto response_received_callback =
      [&](ServiceResponseFuture future) {
        auto request_response_pair = future.get();
        service_finished_ = true;
        returned_value_ = request_response_pair.second->success;
      };

    auto result = client_ptr_->async_send_request(request, std::move(response_received_callback));

    while(!service_finished_){
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    return returned_value_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }


private:
  rclcpp::executors::SingleThreadedExecutor exec_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<ChangeGripperStateService>::SharedPtr  client_ptr_;

  std::string state_;
  double state_numeric_;

  bool service_finished_;
  bool returned_value_;
};
