#pragma once


#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "behaviortree_cpp_v3/action_node.h"
#include <chrono>
#include <thread>
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class StartNN : public BT::SyncActionNode
{
public:
  using SetBool = std_srvs::srv::SetBool;

    StartNN(const std::string& name, const BT::NodeConfiguration& config,
                std::string ns, std::string start_nn_service_name)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("start_nn_bt", ns);
        this->client_ptr_ = node_->create_client<SetBool>(
          start_nn_service_name);

        while(!client_ptr_->wait_for_service(1s)) {
          RCLCPP_INFO(node_->get_logger(), "Client compute_parking_path not available - waiting");
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        RCLCPP_INFO(node_->get_logger(), "StartNN - init");
    }

    static BT::PortsList providedPorts()
    {
        return{};
    }

    virtual BT::NodeStatus tick() override
    {
        nn_started = false;
        returned_value = false;

        auto request = std::make_shared<SetBool::Request>();
        request->data = true;

        using ServiceResponseFuture =
          rclcpp::Client<SetBool>::SharedFutureWithRequest;
        auto response_received_callback =
          [&](ServiceResponseFuture future) {
            auto request_response_pair = future.get();
            this->nn_started = true;
            this->returned_value = request_response_pair.second->success;
          };

        auto result = client_ptr_->async_send_request(request, std::move(response_received_callback));

        while(!this->nn_started){
            rclcpp::spin_some(node_);
        }
        returned_value = true;
        
        return this->returned_value ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }


  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<SetBool>::SharedPtr  client_ptr_;

    bool nn_started;
    bool returned_value;
};
