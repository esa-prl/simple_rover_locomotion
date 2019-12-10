#ifndef LOCOMOTION_MODE_H
#define LOCOMOTION_MODE_H

#include "rclcpp/rclcpp.hpp"

#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>


#include <simple_rover_locomotion/srv/activate.hpp>
#include <simple_rover_locomotion/srv/change_locomotion_mode.hpp>


class LocomotionMode : public rclcpp::Node
{
  public:
    LocomotionMode();

  protected:
    // Joints Pulisher
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr joints_publisher_;
    // Goal Rover Velocities Subscription
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr rover_velocities_subscription_;    
    
    // Velocities Callback
    virtual void rover_velocities_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void initialize_subscribers();


  private:

    // Services Objects
    rclcpp::Service<simple_rover_locomotion::srv::Activate>::SharedPtr activate_service_;
    rclcpp::Service<simple_rover_locomotion::srv::ChangeLocomotionMode>::SharedPtr changelocomotionmode_service_;
 
    // Services Callbacks
    void activate(const simple_rover_locomotion::srv::Activate::Request::SharedPtr request,
            std::shared_ptr<simple_rover_locomotion::srv::Activate::Response>      response);
 
    void change_locomotion_mode(const simple_rover_locomotion::srv::ChangeLocomotionMode::Request::SharedPtr request,
                          std::shared_ptr<simple_rover_locomotion::srv::ChangeLocomotionMode::Response>      response);

    // Parameters
    // string mode_name_;

};

#endif