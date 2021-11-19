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

#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class ActionA: public BT::SyncActionNode
{

public:
    // additional arguments passed to the constructor
    ActionA(const std::string& name, const BT::NodeConfiguration& config,
             int arg1, double arg2, std::string arg3 ):
        SyncActionNode(name, config),
        _arg1(arg1),
        _arg2(arg2),
        _arg3(arg3) {}

    BT::NodeStatus tick() override
    {
        std::cout << "ActionA: " << _arg1 << " / " << _arg2 << " / " << _arg3 << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    static BT::PortsList providedPorts() { return {}; }

private:
    int _arg1;
    double _arg2;
    std::string _arg3;
};
