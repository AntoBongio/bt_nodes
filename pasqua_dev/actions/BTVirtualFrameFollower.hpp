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

#include "controller_interfaces/action/virtual_frame_action.hpp"

class VirtualFrameFollower : public BT::AsyncActionNode
{
public:
  using VirtualFrameAction = controller_interfaces::action::VirtualFrameAction;
  using GoalHandleVirtualFrameAction = rclcpp_action::ClientGoalHandle<VirtualFrameAction>;

    VirtualFrameFollower(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("virtual_frame_follower_bt");
        this->client_ptr_ = rclcpp_action::create_client<VirtualFrameAction>(
          node_,
          "/pasqua_controller/virtual_frame_follower");

        while(!client_ptr_->wait_for_action_server()) {
          RCLCPP_INFO(node_->get_logger(), "Action server virtual frame follower not available after waiting");
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        exec_.add_node(node_);
        std::thread( [&] {this->exec_.spin();} ).detach();

        send_goal_options = rclcpp_action::Client<VirtualFrameAction>::SendGoalOptions();
        send_goal_options.goal_response_callback =
          std::bind(&VirtualFrameFollower::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
          std::bind(&VirtualFrameFollower::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
          std::bind(&VirtualFrameFollower::result_callback, this, std::placeholders::_1);

        RCLCPP_INFO(node_->get_logger(), "VirtualFrameFollower - init");
    }

    static BT::PortsList providedPorts()
    {
        return{ };
    }

    BT::NodeStatus tick() override
    {
        _halt_requested.store(false);

        goal_msg = VirtualFrameAction::Goal();
        parking_maneuver_finished = false;
            
        client_ptr_->async_send_goal(goal_msg, send_goal_options);

        while(not this->parking_maneuver_finished){
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        RCLCPP_INFO(node_->get_logger(), "VirtualFrameFollower - FINISHED");
        return this->returned_value and not _halt_requested ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    void goal_response_callback(std::shared_future<GoalHandleVirtualFrameAction::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
          RCLCPP_INFO(node_->get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(GoalHandleVirtualFrameAction::SharedPtr,
        const std::shared_ptr<const VirtualFrameAction::Feedback> feedback)
    {
        std::stringstream ss;
        ss << "distance: " << feedback->distance;
        RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    }

    void result_callback(const GoalHandleVirtualFrameAction::WrappedResult & result)
    {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_INFO(node_->get_logger(), "Goal was aborted");
            parking_maneuver_finished = true;
            returned_value = false;
            return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(node_->get_logger(), "Goal was canceled");
            parking_maneuver_finished = true;
            returned_value = false;
            return;
          default:
            RCLCPP_INFO(node_->get_logger(), "Unknown result code");
            return;
        }
        // std::stringstream ss;
        // ss << "Result received: " << result.result->ret;
        // RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
        
        parking_maneuver_finished = true;
        returned_value = result.result->ret;
    }

    // This overloaded method is used to stop the execution of this node.
    void halt() override
    {
        RCLCPP_INFO(node_->get_logger(), "VirtualFrameFollower - halt requested");
        parking_maneuver_finished = true;
        returned_value = false;
    }

  private:
    rclcpp::Node::SharedPtr node_;
    controller_interfaces::action::VirtualFrameAction::Goal goal_msg;
    rclcpp_action::Client<VirtualFrameAction>::SharedPtr client_ptr_;
    rclcpp::executors::SingleThreadedExecutor exec_;

    rclcpp_action::Client<VirtualFrameFollower::VirtualFrameAction>::SendGoalOptions send_goal_options;

    std::atomic_bool _halt_requested;
    bool parking_maneuver_finished;
    bool returned_value;
};
