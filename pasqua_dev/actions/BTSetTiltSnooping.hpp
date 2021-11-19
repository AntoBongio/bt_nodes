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

#include "marker_snooping_interfaces/srv/set_tilt_static.hpp"

using namespace std::chrono_literals;

class SetTiltSnooping : public BT::SyncActionNode
{
public:
  using SetTiltStatic = marker_snooping_interfaces::srv::SetTiltStatic;

    SetTiltSnooping(const std::string& name, const BT::NodeConfiguration& config, std::string set_tilt_snooping_service_name)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("set_tilt_static_snoop_bt");
        this->client_ptr_ = node_->create_client<SetTiltStatic>(
          set_tilt_snooping_service_name);

        while(!client_ptr_->wait_for_service(1s)) {
          RCLCPP_INFO(node_->get_logger(), "Client set_tilt_static not available - waiting");
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        this->set_tilt_finished = false;

        RCLCPP_INFO(node_->get_logger(), "SetTiltSnooping - init");
    }

    static BT::PortsList providedPorts()
    {
        return{BT::InputPort<float>("tilt_static") };
    }

    virtual BT::NodeStatus tick() override
    {
        float tilt_static = 0.0;
        if (!getInput<float>("tilt_static", tilt_static)) {
            // if I can't get this, there is something wrong with your BT.
            // For this reason throw an exception instead of returning FAILURE
            throw BT::RuntimeError("missing required input [tilt_static]");
        }

        auto request = std::make_shared<SetTiltStatic::Request>();
        request->tilt_static = tilt_static;

        using ServiceResponseFuture =
          rclcpp::Client<SetTiltStatic>::SharedFutureWithRequest;
        auto response_received_callback =
          [&](ServiceResponseFuture future) {
            auto request_response_pair = future.get();
            this->set_tilt_finished = true;
            this->returned_value = request_response_pair.second->res;
          };

        auto result = client_ptr_->async_send_request(request, std::move(response_received_callback));

        while(!this->set_tilt_finished){
            rclcpp::spin_some(node_);
        }
        
        return this->returned_value ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }


  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<SetTiltStatic>::SharedPtr client_ptr_;
    
    bool set_tilt_finished;
    bool returned_value;
};
