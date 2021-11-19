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

#include "std_srvs/srv/set_bool.hpp"


using namespace std::chrono_literals;

class WaitForUserStart : public BT::AsyncActionNode
{
public:
  using SetBool = std_srvs::srv::SetBool;

    WaitForUserStart(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("wait_for_user_start_bt");

        wait_user_start = node_->create_service<SetBool>("/behaviour_tree/start",
            std::bind(&WaitForUserStart::callback_start, this, std::placeholders::_1, std::placeholders::_2)); 

        this->service_called = false;
        this->user_start = false;

        RCLCPP_INFO(node_->get_logger(), "WaitForUserStart - init");
    }

    void callback_start(const std::shared_ptr<SetBool::Request> request,
        std::shared_ptr<SetBool::Response>      response){
            this->service_called = true;
            this->user_start = request->data;
            response->success = true;
    }

    static BT::PortsList providedPorts()
    {
        return{ };
    }

    virtual BT::NodeStatus tick() override
    {
        _halt_requested.store(false);

        while(not service_called and not _halt_requested){
            rclcpp::spin_some(node_);
        }
        
        return this->user_start and not _halt_requested ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    // This overloaded method is used to stop the execution of this node.
    void halt() override
    {
         _halt_requested.store(true);
    }


  private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Service<SetBool>::SharedPtr wait_user_start;
    rclcpp::executors::SingleThreadedExecutor exec_;

    std::atomic_bool _halt_requested;
    bool user_start;
    bool service_called;
    bool returned_value;
};
