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
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class DeployArm : public BT::SyncActionNode
{
public:
  using Trigger = std_srvs::srv::Trigger;

    DeployArm(const std::string& name, const BT::NodeConfiguration& config,
                std::string ns, std::string deploy_arm_service_name)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("deploy_arm_bt", ns);
        this->client_ptr_ = node_->create_client<Trigger>(
          deploy_arm_service_name);

        // while(!client_ptr_->wait_for_service(1s)) {
        //   RCLCPP_INFO(node_->get_logger(), "Client deploy_arm not available - waiting");
        //   std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // }

        RCLCPP_INFO(node_->get_logger(), "DeployArm - init");
    }

    static BT::PortsList providedPorts()
    {
        return{};
    }

    virtual BT::NodeStatus tick() override
    {
        arm_deployed = false;
        returned_value = false;

        // auto request = std::make_shared<Trigger::Request>();

        // using ServiceResponseFuture =
        //   rclcpp::Client<Trigger>::SharedFutureWithRequest;
        // auto response_received_callback =
        //   [&](ServiceResponseFuture future) {
        //     auto request_response_pair = future.get();
        //     this->arm_deployed = true;
        //     this->returned_value = request_response_pair.second->success;
        //   };

        // auto result = client_ptr_->async_send_request(request, std::move(response_received_callback));

        // while(!this->arm_deployed){
        //     rclcpp::spin_some(node_);
        // }
        returned_value = true;
        
        return this->returned_value ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }


  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<Trigger>::SharedPtr  client_ptr_;

    bool arm_deployed;
    bool returned_value;
};
