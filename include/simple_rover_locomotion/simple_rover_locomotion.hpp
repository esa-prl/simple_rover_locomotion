#ifndef SIMPLE_ROVER_LOCOMOTION_H
#define SIMPLE_ROVER_LOCOMOTION_H

#include "locomotion_mode/locomotion_mode.hpp"


class SimpleRoverLocomotion: public LocomotionMode
{
  public:
    SimpleRoverLocomotion(rclcpp::NodeOptions options, std::string node_name);

  private:
    void rover_velocities_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
};

#endif
