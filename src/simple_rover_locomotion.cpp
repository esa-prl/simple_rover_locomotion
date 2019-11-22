#include "simple_rover_locomotion/simple_rover_locomotion.hpp"


SimpleRoverLocomotion::SimpleRoverLocomotion()
: Node("simple_rover_locomotion")
{
  joints_publisher_ = this->create_publisher<std_msgs::msg::String>("~/joint_commands", 10);
  rover_velocities_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
  "~/rover_velocities", 10, std::bind(&SimpleRoverLocomotion::rover_velocities_callback, this, std::placeholders::_1));

  service_ = this->create_service<simple_rover_locomotion::srv::Activate>("~/activate", std::bind(&SimpleRoverLocomotion::activate, this, std::placeholders::_1, std::placeholders::_2));


}

void SimpleRoverLocomotion::rover_velocities_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

  RCLCPP_INFO(this->get_logger(), "I heard: %d.", msg->twist.linear.x);
}

void SimpleRoverLocomotion::activate(const simple_rover_locomotion::srv::Activate::Request::SharedPtr request,
         std::shared_ptr<simple_rover_locomotion::srv::Activate::Response>      response)
{
    if (request->goal_state) {
        response->new_state = true;
    }
    else {response->new_state = false;}

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal State %d", request->goal_state);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "New State %d", response->new_state);
}


int main(int argc, char * argv[])
{
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<SimpleRoverLocomotion>());
rclcpp::shutdown();
return 0;
}