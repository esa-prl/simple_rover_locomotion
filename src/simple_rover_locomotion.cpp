#include "simple_rover_locomotion/simple_rover_locomotion.hpp"

SimpleRoverLocomotion::SimpleRoverLocomotion()
{
  // Create Subscription and callback to derived class method
  this->initialize_subscribers();

  RCLCPP_INFO(this->get_logger(), "SimpleRoverLocomotion started");

}

void SimpleRoverLocomotion::rover_velocities_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "X_linear: %f.", msg->linear.x);
  RCLCPP_INFO(this->get_logger(), "Y_linear: %f.", msg->linear.y);
  RCLCPP_INFO(this->get_logger(), "Z_linear: %f.", msg->linear.z);
  RCLCPP_INFO(this->get_logger(), "X_angular: %f.", msg->angular.x);
  RCLCPP_INFO(this->get_logger(), "Y_angular: %f.", msg->angular.y);
  RCLCPP_INFO(this->get_logger(), "Z_angular: %f.", msg->angular.z);

  RCLCPP_INFO(this->get_logger(), "In Simple Rover rover_velocities_callback");

  double x_dot = msg->linear.x;
  double y_dot = msg->linear.y;
  double theta_dot = msg->angular.z;

  double lower_position_limit = -100;   // -100 deg
  double upper_position_limit = -lower_position_limit;    // +100 deg

  RCLCPP_INFO(this->get_logger(), "\t\t\t\tMOTOR NAME: %s", current_joint_state_.name[0].c_str()); 

}




int main(int argc, char * argv[])
{
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<SimpleRoverLocomotion>());
rclcpp::shutdown();
return 0;
}