#ifndef SIMPLE_ROVER_LOCOMOTION_H
#define SIMPLE_ROVER_LOCOMOTION_H

#include "locomotion_mode/locomotion_mode.hpp"

namespace locomotion_mode {  

  class SimpleRoverLocomotion : public LocomotionMode
  {
  public:
    SimpleRoverLocomotion(rclcpp::NodeOptions options, std::string node_name);

  private:
    // Position of the centre of rotation in x and y
    double centre_of_rotation_x_;
    double centre_of_rotation_y_;
    // 'True' if all wheels are steerable
    bool fully_steerable_;

    // Margin in whicc the driving wheels start driving. [rad]
    double steering_margin_;
    bool steering_in_progress_;

    void rover_velocities_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    // Checks if all wheels are steerable. At most, two wheels can be non steerable, otherwise it fails.
    bool check_steering_limitations();

    // Uncomment for enable overwrite example
    // bool enabling_sequence();
    
  };
}

#endif
