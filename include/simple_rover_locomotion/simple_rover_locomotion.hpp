#ifndef SIMPLE_ROVER_LOCOMOTION_TURTLE_H
#define SIMPLE_ROVER_LOCOMOTION_TURTLE_H


#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <simple_rover_locomotion/srv/activate.hpp>


class SimpleRoverLocomotion : public rclcpp::Node
{
  public:
    SimpleRoverLocomotion();

  private:
    void rover_velocities_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr joints_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr rover_velocities_subscription_;
    rclcpp::Service<simple_rover_locomotion::srv::Activate>::SharedPtr service_;
 
    void activate(const simple_rover_locomotion::srv::Activate::Request::SharedPtr request,
             std::shared_ptr<simple_rover_locomotion::srv::Activate::Response>      response);

};

#endif
