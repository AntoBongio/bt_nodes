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

class SetParkingMode : public BT::SyncActionNode
{
public:
  using SetBool = std_srvs::srv::SetBool;

    SetParkingMode(const std::string& name, const BT::NodeConfiguration& config, std::string set_parking_mode_service_name)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("set_parking_mode_bt");
        this->client_ptr_ = node_->create_client<SetBool>(
          set_parking_mode_service_name
          );

        while(!client_ptr_->wait_for_service(1s)) {
          RCLCPP_INFO(node_->get_logger(), "Client set_parking_mode not available - waiting");
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        this->parking_mode_set = false;

        RCLCPP_INFO(node_->get_logger(), "SetParkingMode - init");
    }

    static BT::PortsList providedPorts()
    {
        return{BT::InputPort<bool>("mode") };
    }

    virtual BT::NodeStatus tick() override
    {
        bool parking_mode = true;
        if (!getInput<bool>("mode", parking_mode)) {
            // if I can't get this, there is something wrong with your BT.
            // For this reason throw an exception instead of returning FAILURE
            throw BT::RuntimeError("missing required input [mode]");
        }

        returned_value = false;
        auto request = std::make_shared<SetBool::Request>();
        request->data = parking_mode;

        using ServiceResponseFuture =
          rclcpp::Client<SetBool>::SharedFutureWithRequest;
        auto response_received_callback =
          [&](ServiceResponseFuture future) {
            auto request_response_pair = future.get();
            this->parking_mode_set = true;
            this->returned_value = request_response_pair.second->success;
          };

        auto result = client_ptr_->async_send_request(request, std::move(response_received_callback));

        while(!this->parking_mode_set){
            rclcpp::spin_some(node_);
        }
        
        return this->returned_value ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }


  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<SetBool>::SharedPtr  client_ptr_;

    bool parking_mode_set;
    bool returned_value;
};
