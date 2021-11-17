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

#include "controller_interfaces/action/spin.hpp"

class SpinManeuver : public BT::AsyncActionNode
{
public:
  using Spin = controller_interfaces::action::Spin;
  using GoalHandleSpin = rclcpp_action::ClientGoalHandle<Spin>;

    SpinManeuver(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("spin_maneuver_bt");
        this->client_ptr_ = rclcpp_action::create_client<Spin>(
          node_,
          "/pasqua_controller/spin");

        while(!client_ptr_->wait_for_action_server()) {
          RCLCPP_INFO(node_->get_logger(), "Action server spin not available after waiting");
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        exec_.add_node(node_);
        std::thread( [&] {this->exec_.spin();} ).detach();

        send_goal_options = rclcpp_action::Client<Spin>::SendGoalOptions();
        send_goal_options.goal_response_callback =
          std::bind(&SpinManeuver::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
          std::bind(&SpinManeuver::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
          std::bind(&SpinManeuver::result_callback, this, std::placeholders::_1);

        RCLCPP_INFO(node_->get_logger(), "SpinManeuver - init");
    }

    static BT::PortsList providedPorts()
    {
        return{BT::InputPort<double>("rad_angle") };
    }

    virtual BT::NodeStatus tick() override
    {
        _halt_requested.store(false);

        double spin_angle_rad = 0.0;
        if (!getInput<double>("rad_angle", spin_angle_rad)) {
            // if I can't get this, there is something wrong with your BT.
            // For this reason throw an exception instead of returning FAILURE
            throw BT::RuntimeError("missing required input [rad_angle]");
        }

        goal_msg = Spin::Goal();
        goal_msg.rad_angle = spin_angle_rad;
        
        client_ptr_->async_send_goal(goal_msg, send_goal_options);

        while(!this->parking_maneuver_finished){
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        RCLCPP_INFO(node_->get_logger(), "SpinManeuver - FINISHED");
        return _halt_requested or not this->returned_value ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }

    // This overloaded method is used to stop the execution of this node.
    void halt() override
    {
        _halt_requested.store(true);
    }

    void goal_response_callback(std::shared_future<GoalHandleSpin::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
          RCLCPP_INFO(node_->get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(GoalHandleSpin::SharedPtr,
        const std::shared_ptr<const Spin::Feedback> feedback)
    {
        // std::stringstream ss;
        // ss << "percentage_of_completing: " << feedback->percentage_of_completing;
        // RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    }

    void result_callback(const GoalHandleSpin::WrappedResult & result)
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
        std::stringstream ss;
        ss << "Result received: " << result.result->ret;
        RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

        parking_maneuver_finished = true;
        returned_value = result.result->ret;
    }

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<Spin>::SharedPtr client_ptr_;
    rclcpp::executors::SingleThreadedExecutor exec_;

    controller_interfaces::action::Spin::Goal goal_msg;
    rclcpp_action::Client<SpinManeuver::Spin>::SendGoalOptions send_goal_options;

    std::atomic_bool _halt_requested;
    bool parking_maneuver_finished;
    bool returned_value;
};
