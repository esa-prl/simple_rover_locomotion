#ifndef SIMPLE_ROVER_LOCOMOTION_H
#define SIMPLE_ROVER_LOCOMOTION_H

#include <simple_rover_locomotion/locomotion_mode.hpp>


// class SimpleRoverLocomotion : public rclcpp::Node
class SimpleRoverLocomotion: public LocomotionMode
{
  public:
    SimpleRoverLocomotion(rclcpp::NodeOptions options);

  private:
    void rover_velocities_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
};

#endif
