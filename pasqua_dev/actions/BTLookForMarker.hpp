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

#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

class LookForMarker : public BT::SyncActionNode
{
public:
  using Empty = std_srvs::srv::Empty;

    LookForMarker(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("look_for_marker_bt");
        this->client_ptr_ = node_->create_client<Empty>(
          "/parking/start_searching");

        while(!client_ptr_->wait_for_service(1s)) {
          RCLCPP_INFO(node_->get_logger(), "Client start_searching not available - waiting");
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        RCLCPP_INFO(node_->get_logger(), "LookForMarker - init");
    }

    static BT::PortsList providedPorts()
    {
        return{ };
    }

    virtual BT::NodeStatus tick() override
    {
        search_finished = false;
        auto request = std::make_shared<Empty::Request>();

        using ServiceResponseFuture =
          rclcpp::Client<Empty>::SharedFutureWithRequest;
        auto response_received_callback =
          [&](ServiceResponseFuture future) {
            auto request_response_pair = future.get();
            this->search_finished = true;
          };

        auto result = client_ptr_->async_send_request(request, std::move(response_received_callback));

        while(!this->search_finished){
            rclcpp::spin_some(node_);
        }
        
        return BT::NodeStatus::SUCCESS;
    }




  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<Empty>::SharedPtr  client_ptr_;
    
    bool search_finished;
};
