#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>

using namespace std::chrono_literals;

class FollowTarget: public BT::AsyncActionNode
{
public:
  FollowTarget(const std::string& name, const BT::NodeConfiguration& config)
      : BT::AsyncActionNode(name, config)
    {
      node_ = rclcpp::Node::make_shared("FollowTarget");

      follow_target_ = false;
      pub_follow_target_ = this->node_->create_publisher<std_msgs::msg::Bool>("/follow_target", 1);
      double hz = 50.0;
      timer_follow_target_ = this->node_->create_wall_timer(1000ms / hz, 
        [this]() {
          auto msg = std_msgs::msg::Bool();
          msg.set__data(follow_target_);
          this->pub_follow_target_->publish(msg);
        });
      
      exec_.add_node(node_);
      std::thread( [&] {this->exec_.spin();} ).detach();
    };

  static BT::PortsList providedPorts()
  {
      return { };
  };

  virtual BT::NodeStatus tick() override 
  {
    if(!follow_target_) {
      start_time_ = std::chrono::system_clock::now();
      follow_target_ = true;
    }
    end_time_ = std::chrono::system_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time_ - start_time_).count();

    std::cout << std::endl << "duration: " << duration << ", follow_target_: " << follow_target_ << std::endl;
    while(duration <= 20.0) {
      end_time_ = std::chrono::system_clock::now();
      duration = std::chrono::duration_cast<std::chrono::seconds>(end_time_ - start_time_).count();

      if(duration >= 20.0) {
        follow_target_ = false;
        return BT::NodeStatus::SUCCESS;
      }  
    }
  };

  void halt() override {
    std::cout << std::endl << "[FollowTarget] HALT!!!!" << std::endl;
  }


private:
  rclcpp::executors::SingleThreadedExecutor exec_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_follow_target_;
  rclcpp::TimerBase::SharedPtr timer_follow_target_;
  bool follow_target_;

  std::chrono::time_point<std::chrono::system_clock> start_time_;
  std::chrono::time_point<std::chrono::system_clock> end_time_;

};