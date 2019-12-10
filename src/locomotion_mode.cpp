#include "simple_rover_locomotion/locomotion_mode.hpp"

LocomotionMode::LocomotionMode()
: Node("locomotion_mode_node")
{
  joints_publisher_ = this->create_publisher<std_msgs::msg::String>("rover_joint_cmds", 10);

  activate_service_ = this->create_service<simple_rover_locomotion::srv::Activate>("activate", std::bind(&LocomotionMode::activate, this, std::placeholders::_1, std::placeholders::_2));
  changelocomotionmode_service_ = this->create_service<simple_rover_locomotion::srv::ChangeLocomotionMode>("change_locomotion_mode", std::bind(&LocomotionMode::change_locomotion_mode, this, std::placeholders::_1, std::placeholders::_2));
  
  RCLCPP_INFO(this->get_logger(), "LocomotionMode initialized");

}

void LocomotionMode::initialize_subscribers()
{
  rover_velocities_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
  "rover_motion_cmd", 10, std::bind(&LocomotionMode::rover_velocities_callback, this, std::placeholders::_1));
}

void LocomotionMode::activate(const simple_rover_locomotion::srv::Activate::Request::SharedPtr request,
         std::shared_ptr<simple_rover_locomotion::srv::Activate::Response>      response)
{
    if (request->goal_state) {
        response->new_state = true;
    }
    else {response->new_state = false;}

    RCLCPP_INFO(this->get_logger(), "Goal State %d", request->goal_state);
    RCLCPP_INFO(this->get_logger(), "New State %d", response->new_state);
}

void LocomotionMode::change_locomotion_mode(const simple_rover_locomotion::srv::ChangeLocomotionMode::Request::SharedPtr request,
         std::shared_ptr<simple_rover_locomotion::srv::ChangeLocomotionMode::Response>      response)
{
    // if (request->locomotion_mode == '2D_KINEMATICS' || request->locomotion_mode == 'WHEELWALKING') {
    //     response->response = 'LOCOMOTION CHANGED';
    // }
    // else {response->response = 'INVALID LOCOMOTION MODE REQUESTED';}

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response: %s", response->response.c_str());
    response->response = "SUCCSEFULLY SET " + request->locomotion_mode + " MODE!";
    RCLCPP_INFO(this->get_logger(), "Desired Locomotion Mode %s", request->locomotion_mode.c_str());
}

void LocomotionMode::rover_velocities_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  RCLCPP_INFO(this->get_logger(), "X_linear: %f.", msg->linear.x);
  RCLCPP_INFO(this->get_logger(), "Y_linear: %f.", msg->linear.y);
  RCLCPP_INFO(this->get_logger(), "Z_linear: %f.", msg->linear.z);
  RCLCPP_INFO(this->get_logger(), "X_angular: %f.", msg->angular.x);
  RCLCPP_INFO(this->get_logger(), "Y_angular: %f.", msg->angular.y);

  RCLCPP_WARN(this->get_logger(), "Rover Velocities Callback was not overridden!");
}