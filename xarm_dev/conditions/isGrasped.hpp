#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <behaviortree_cpp_v3/condition_node.h>

using namespace std::chrono_literals;

class isGrasped: public BT::ConditionNode 
{
  public:
    isGrasped(const std::string& name, const BT::NodeConfiguration& config, std::string & ns, std::string topic_right_finger, std::string topic_left_finger)
        : BT::ConditionNode(name, config)
      {
        node_ = rclcpp::Node::make_shared("is_grasped", ns);

        right_finger_grasped_ = false;
        left_finger_grasped_ = false;

        sub_right_finger_ = node_->create_subscription<std_msgs::msg::Int32>(
          topic_right_finger,
          1,
          [&](std_msgs::msg::Int32::UniquePtr msg) {
            if(msg->data >= 30) {
              right_finger_grasped_ = true;
            }
            else {
              right_finger_grasped_ = false;
            }
          });

        sub_left_finger_ = node_->create_subscription<std_msgs::msg::Int32>(
          topic_left_finger,
          1,
          [&](std_msgs::msg::Int32::UniquePtr msg) {
            if(msg->data >= 30){
              left_finger_grasped_ = true;
            }
            else {
              left_finger_grasped_ = false;
            }
          });

        RCLCPP_INFO(node_->get_logger(), "is Grasped -  init");
        
        exec_.add_node(node_);
        std::thread( [&] {this->exec_.spin();} ).detach();
      }

    static BT::PortsList providedPorts()
    {
        return { };
    }

    virtual BT::NodeStatus tick() override 
    {
      return (left_finger_grasped_ and right_finger_grasped_) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }


  private:
    rclcpp::executors::SingleThreadedExecutor exec_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_right_finger_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_left_finger_;

    std::atomic_bool left_finger_grasped_;
    std::atomic_bool right_finger_grasped_;
};