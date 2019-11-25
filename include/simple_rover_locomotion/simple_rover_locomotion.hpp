#ifndef SIMPLE_ROVER_LOCOMOTION_TURTLE_H
#define SIMPLE_ROVER_LOCOMOTION_TURTLE_H


#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <simple_rover_locomotion/srv/activate.hpp>
#include <simple_rover_locomotion/srv/change_locomotion_mode.hpp>


class SimpleRoverLocomotion : public rclcpp::Node
{
  public:
    SimpleRoverLocomotion();

  private:
    void rover_velocities_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr joints_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr rover_velocities_subscription_;
    rclcpp::Service<simple_rover_locomotion::srv::Activate>::SharedPtr activate_service_;
    rclcpp::Service<simple_rover_locomotion::srv::ChangeLocomotionMode>::SharedPtr changelocomotionmode_service_;
 
    void activate(const simple_rover_locomotion::srv::Activate::Request::SharedPtr request,
             std::shared_ptr<simple_rover_locomotion::srv::Activate::Response>      response);
 
    void change_locomotion_mode(const simple_rover_locomotion::srv::ChangeLocomotionMode::Request::SharedPtr request,
             std::shared_ptr<simple_rover_locomotion::srv::ChangeLocomotionMode::Response>      response);

};

#endif
