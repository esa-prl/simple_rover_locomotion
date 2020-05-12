#ifndef SIMPLE_ROVER_LOCOMOTION_H
#define SIMPLE_ROVER_LOCOMOTION_H

#include "locomotion_mode/locomotion_mode.hpp"
#include "rclcpp/clock.hpp"

class SimpleRoverLocomotion : public LocomotionMode
{
public:
  SimpleRoverLocomotion(rclcpp::NodeOptions options, std::string node_name);

private:
  // Position of the centre of rotation in x and y
  double centre_of_rotation_x;
  double centre_of_rotation_y;
  // 'True' if all wheels are steerable
  bool fully_steerable_;

  // Margin in whicc the driving wheels start driving. [rad]
  double steering_margin_;
  bool steering_in_progress_;

  void rover_velocities_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  bool check_steering_limitations();
  // bool enable();
};

#endif
