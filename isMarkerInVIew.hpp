#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <behaviortree_cpp_v3/action_node.h>

using namespace std::chrono_literals;

class isMarkerInView: public BT::ConditionNode 
{
  public:
    isMarkerInView(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config)
      {
        node_ = rclcpp::Node::make_shared("isMarkerInView");

        in_view_ = false;
        sub_in_view_ = this->node_->create_subscription<std_msgs::msg::Bool>(
          "/is_in_view",
          1,
          [&](std_msgs::msg::Bool::UniquePtr msg) {
            in_view_ = msg->data;
          });
        
        exec_.add_node(node_);
        std::thread( [&] {this->exec_.spin();} ).detach();
      }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("id") };
    }

    virtual BT::NodeStatus tick() override 
    {
      return in_view_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }


  private:
    rclcpp::executors::SingleThreadedExecutor exec_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_in_view_;
    bool in_view_;
};