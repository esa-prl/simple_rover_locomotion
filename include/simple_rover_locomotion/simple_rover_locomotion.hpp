#pragma once

#include "locomotion_mode/locomotion_mode.hpp"

namespace locomotion_mode {  

class SimpleRoverLocomotion : public LocomotionMode
{
public:
  SimpleRoverLocomotion(rclcpp::NodeOptions options, std::string node_name);

private:
  // Offset of origin x and y
  double origin_offset_x_;
  double origin_offset_y_;
  // 'True' if all wheels are steerable
  bool fully_steerable_;

  // Margin in whicc the driving wheels start driving. [rad]
  double steering_margin_;
  bool steering_in_progress_;

  rover_msgs::msg::JointCommandArray compute_joint_commands(
    const geometry_msgs::msg::Twist::SharedPtr msg);

  // Checks if all wheels are steerable. At most, two wheels can be non steerable, otherwise it fails.
  bool check_steering_limitations();

  // Uncomment for enable overwrite example
  // bool enabling_sequence();
  
};

}
