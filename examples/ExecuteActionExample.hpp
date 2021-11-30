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

#include "action_tutorials_interfaces/action/fibonacci.hpp"

class ExecuteActionExample : public BT::AsyncActionNode
{
public:
  using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

    ExecuteActionExample(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("execute_action_example_bt");
        this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
          node_,
          "/action_server_name");


        while(!client_ptr_->wait_for_action_server()) {
          RCLCPP_INFO(node_->get_logger(), "Action server not available after waiting");
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        exec_.add_node(node_);
        std::thread( [&] {this->exec_.spin();} ).detach();

        send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
        send_goal_options.goal_response_callback =
          std::bind(&ExecuteActionExample::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
          std::bind(&ExecuteActionExample::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
          std::bind(&ExecuteActionExample::result_callback, this, std::placeholders::_1);
          
        RCLCPP_INFO(node_->get_logger(), "ExecuteActionExample - init");
    }

    static BT::PortsList providedPorts()
    {
        return{ };
    }

    virtual BT::NodeStatus tick() override
    {
        goal_msg = Fibonacci::Goal();

        _halt_requested.store(false);
        action_finished = false;
        
        client_ptr_->async_send_goal(goal_msg, send_goal_options);

        while(not this->action_finished){
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        RCLCPP_INFO(node_->get_logger(), "ExecuteActionExample - FINISHED");
        return this->returned_value and not _halt_requested ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    void goal_response_callback(std::shared_future<GoalHandleFibonacci::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
          RCLCPP_INFO(node_->get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(GoalHandleFibonacci::SharedPtr,
        const std::shared_ptr<const Fibonacci::Feedback>)
    {
        
    }

    void result_callback(const GoalHandleFibonacci::WrappedResult & result)
    {
      if (not _halt_requested)
      {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_INFO(node_->get_logger(), "Goal was aborted");
            action_finished = true;
            returned_value = false;
            return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(node_->get_logger(), "Goal was canceled");
            action_finished = true;
            returned_value = false;
            return;
          default:
            RCLCPP_INFO(node_->get_logger(), "Unknown result code");
            return;
        }
        // std::stringstream ss;
        // ss << "Result received: " << result.result->sequence;
        // RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
        
        action_finished = true;
        returned_value = true;
      }
    }

    // This overloaded method is used to stop the execution of this node.
    void halt() override
    {
        RCLCPP_INFO(node_->get_logger(), "ExecuteActionExample - halt requested");
        action_finished = true;
        returned_value = false;
        _halt_requested.store(true);
    }

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
    rclcpp::executors::SingleThreadedExecutor exec_;

    rclcpp_action::Client<ExecuteActionExample::Fibonacci>::SendGoalOptions send_goal_options;
    Fibonacci::Goal goal_msg;

    std::atomic_bool _halt_requested;
    bool action_finished;
    bool returned_value;
};
