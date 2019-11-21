#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


class SimpleRoverLocomotion : public rclcpp::Node
{
  public:
    SimpleRoverLocomotion()
    : Node("simple_rover_locomotion")
    {
      joints_publisher_ = this->create_publisher<std_msgs::msg::String>("~/joint_commands", 10);
      rover_velocities_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "~/rover_velocities", 10, std::bind(&SimpleRoverLocomotion::rover_velocities_callback, this, std::placeholders::_1));

    }

  private:
    void rover_velocities_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr joints_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rover_velocities_subscription_;

  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleRoverLocomotion>());
    rclcpp::shutdown();
    return 0;
  }