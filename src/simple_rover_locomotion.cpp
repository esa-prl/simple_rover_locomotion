#include "simple_rover_locomotion/simple_rover_locomotion.hpp"

SimpleRoverLocomotion::SimpleRoverLocomotion(rclcpp::NodeOptions options) : LocomotionMode(options)
{
  // Create Subscription and callback to derived class method
  this->initialize_subscribers();

  RCLCPP_WARN(this->get_logger(), "SimpleRoverLocomotion started");

}

void SimpleRoverLocomotion::rover_velocities_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "X_linear: %f.", msg->linear.x);
  RCLCPP_DEBUG(this->get_logger(), "Y_linear: %f.", msg->linear.y);
  RCLCPP_DEBUG(this->get_logger(), "Z_linear: %f.", msg->linear.z);
  RCLCPP_DEBUG(this->get_logger(), "X_angular: %f.", msg->angular.x);
  RCLCPP_DEBUG(this->get_logger(), "Y_angular: %f.", msg->angular.y);
  RCLCPP_DEBUG(this->get_logger(), "Z_angular: %f.", msg->angular.z);

  // double x_dot = msg->linear.x;
  // double y_dot = msg->linear.y;
  // double theta_dot = msg->angular.z;

  // double lower_position_limit = -100;   // -100 deg
  // double upper_position_limit = -lower_position_limit;    // +100 deg

  // It's important to check if the message was already initialized
  // TODO: Check if Joint State is older than a certain age and if send a warning/refuse execution accordingly
  for (size_t i = 0; i < current_joint_state_.name.size(); i++)
  {
    RCLCPP_INFO(this->get_logger(), "\t\t\t\tMOTOR NAME: %s", current_joint_state_.name[i].c_str());
  }

}


int main(int argc, char * argv[])
{
rclcpp::NodeOptions options;
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<SimpleRoverLocomotion>(options));
rclcpp::shutdown();
return 0;
}