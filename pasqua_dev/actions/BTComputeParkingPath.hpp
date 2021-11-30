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

using namespace std::chrono_literals;

class ComputeParkingPath : public BT::SyncActionNode
{
public:
  using Trigger = std_srvs::srv::Trigger;

    ComputeParkingPath(const std::string& name, const BT::NodeConfiguration& config, 
          std::string ns, std::string compute_parking_path_service_name)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("compute_parking_path_bt", ns);
        this->client_ptr_ = node_->create_client<Trigger>(
          compute_parking_path_service_name);

        while(!client_ptr_->wait_for_service(1s)) {
          RCLCPP_INFO(node_->get_logger(), "Client compute_parking_path not available - waiting");
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        RCLCPP_INFO(node_->get_logger(), "ComputeParkingPath - init");
    }

    static BT::PortsList providedPorts()
    {
        return{};
    }

    virtual BT::NodeStatus tick() override
    {
        parking_path_computed = false;
        returned_value = false;

        auto request = std::make_shared<Trigger::Request>();

        using ServiceResponseFuture =
          rclcpp::Client<Trigger>::SharedFutureWithRequest;
        auto response_received_callback =
          [&](ServiceResponseFuture future) {
            auto request_response_pair = future.get();
            this->parking_path_computed = true;
            this->returned_value = request_response_pair.second->success;
          };

        auto result = client_ptr_->async_send_request(request, std::move(response_received_callback));

        while(!this->parking_path_computed){
            rclcpp::spin_some(node_);
        }
        
        return this->returned_value ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }




  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<Trigger>::SharedPtr  client_ptr_;

    bool parking_path_computed;
    bool returned_value;
};
