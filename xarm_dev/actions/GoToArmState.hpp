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

#include "xarm_test_moveit_interfaces/srv/go_to_arm_state.hpp"


using namespace std::chrono_literals;

#define STATE_MAX 850
#define STATE_MIN 0

class GoToArmState : public BT::SyncActionNode
{
public:
  using GoToArmStateService = xarm_test_moveit_interfaces::srv::GoToArmState;

  GoToArmState(const std::string& name, const BT::NodeConfiguration& config, std::string & ns)
      : BT::SyncActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("go_to_arm_state", ns);
    this->client_ptr_ = node_->create_client<GoToArmStateService>(
      "/go_to_arm_state");

    while(!client_ptr_->wait_for_service(1s)) {
      RCLCPP_INFO(node_->get_logger(), "Client go_to_arm_state not available - waiting");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    state_ = "";
    if (!getInput<std::string>("state", state_)) {
    // if I can't get this, there is something wrong with your BT.
    // For this reason throw an exception instead of returning FAILURE
    throw BT::RuntimeError("missing required input [state]");
    }

    RCLCPP_INFO(node_->get_logger(), "Go To Arm State - init");

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

    auto request = std::make_shared<GoToArmStateService::Request>();

    request->state.data = state_;

    using ServiceResponseFuture =
      rclcpp::Client<GoToArmStateService>::SharedFutureWithRequest;
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
  rclcpp::Client<GoToArmStateService>::SharedPtr  client_ptr_;

  std::string state_;

  bool service_finished_;
  bool returned_value_;
};
