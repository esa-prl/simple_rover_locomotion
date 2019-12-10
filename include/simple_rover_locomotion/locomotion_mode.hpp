#ifndef LOCOMOTION_MODE_H
#define LOCOMOTION_MODE_H

#include "rclcpp/rclcpp.hpp"

#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <simple_rover_locomotion/msg/joint_command.hpp>
#include <simple_rover_locomotion/msg/joint_command_array.hpp>

#include <simple_rover_locomotion/srv/activate.hpp>
#include <simple_rover_locomotion/srv/change_locomotion_mode.hpp>


class LocomotionMode : public rclcpp::Node
{
  public:
    LocomotionMode();

  protected:
    // Joints Pulisher
    rclcpp::Publisher<simple_rover_locomotion::msg::JointCommandArray>::SharedPtr joints_publisher_;

    // Goal Rover Velocities Subscription
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr rover_velocities_subscription_;    
    // Velocities Callback
    virtual void rover_velocities_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // Initialize Subscriber with callback function from derived class
    void initialize_subscribers();

    // Initialize Messages
    // simple_rover_locomotion::msg::JointCommand joint_command_;
    simple_rover_locomotion::msg::JointCommandArray joint_command_array_;

    sensor_msgs::msg::JointState current_joint_state_;

  private:

    // Services Objects
    rclcpp::Service<simple_rover_locomotion::srv::Activate>::SharedPtr activate_service_;
    rclcpp::Service<simple_rover_locomotion::srv::ChangeLocomotionMode>::SharedPtr changelocomotionmode_service_;
 
    // Services Callbacks
    void activate(const simple_rover_locomotion::srv::Activate::Request::SharedPtr request,
            std::shared_ptr<simple_rover_locomotion::srv::Activate::Response>      response);
    void change_locomotion_mode(const simple_rover_locomotion::srv::ChangeLocomotionMode::Request::SharedPtr request,
                          std::shared_ptr<simple_rover_locomotion::srv::ChangeLocomotionMode::Response>      response);

    // Joint States Callback
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);


    // Parameters
    // string mode_name_;

};

#endif