#include "simple_rover_locomotion/simple_rover_locomotion.hpp"
#include "rclcpp/clock.hpp"

namespace locomotion_mode{

SimpleRoverLocomotion::SimpleRoverLocomotion(rclcpp::NodeOptions options, std::string node_name)
: LocomotionMode(options, node_name),
// Assuming all rovers are limited except if they show to be unlimited.
  fully_steerable_(false),
  steering_margin_(30 * M_PI / 180),
  steering_in_progress_(false)
{
  if (!check_steering_limitations()) {
    RCLCPP_ERROR(
      this->get_logger(), "%s not able to work, since the robot model has an invalid steering configuration.",
      node_name.c_str());
  }
}


bool SimpleRoverLocomotion::check_steering_limitations()
{
  std::vector<std::shared_ptr<Rover::Leg>> non_steerable_legs;

  // Finds the non_steerable legs and saves them.
  for (auto leg : rover_->legs_) {
    // Check if steering motor joint is populated
    if (!leg->is_steerable())
    {
      non_steerable_legs.push_back(leg);
    }
  }

  // Checks how many legs are non steerable and sets the centre of rotation accortingly
  if (non_steerable_legs.size() == 0) {
    RCLCPP_INFO(this->get_logger(), "All wheels are steerable");
    centre_of_rotation_x_ = 0;
    centre_of_rotation_y_ = 0;
    // Overwrite limited limited steerability
    fully_steerable_ = true;
  }
  else if (non_steerable_legs.size() == 1)
  {
    centre_of_rotation_x_ = non_steerable_legs[0]->driving_motor->global_pose.position.x;
    // Can be any value. 0 makes the most sense.
    centre_of_rotation_y_ = 0;
  }
  else if (non_steerable_legs.size() == 2)
  {
    RCLCPP_WARN(this->get_logger(), "Number of non steerable wheels: %u", non_steerable_legs.size());
    // In case there are multiple fixed wheels their axes must align
    // Check if their axes align with a margin of 1 cm
    if (abs(
        non_steerable_legs[0]->driving_motor->global_pose.position.x -
        non_steerable_legs[1]->driving_motor->global_pose.position.x) > 0.01)
    {
      RCLCPP_ERROR(this->get_logger(), "Non steerable wheels don't align.");
      return false;
    }
    // Since they align set the centre of rotation between them.
    centre_of_rotation_x_ = (non_steerable_legs[0]->driving_motor->global_pose.position.x +
      non_steerable_legs[1]->driving_motor->global_pose.position.x) / 2;
    // This should be 0 in most cases
    centre_of_rotation_y_ = (non_steerable_legs[0]->driving_motor->global_pose.position.y +
      non_steerable_legs[1]->driving_motor->global_pose.position.y) / 2;
  }
  else {
    RCLCPP_ERROR(this->get_logger(), "MORE THAN TWO NON STEERABLE LEGS");
    return false;
  }

  return true;
}

// See the example bellow how to add a custom transition.
// // Don't forget to define it in the .hpp file as well.
// bool SimpleRoverLocomotion::enabling_sequence(){
//   // Testing of custom transition
//   RCLCPP_INFO(this->get_logger(), "ENABLING SimpleRoverLocomotion using a custom transition");
//   // Transition via a hard coded pose
//   return transition_to_robot_pose("straight");
//   // Transition via the pose specified in the config file.
//   // return transition_to_robot_pose(enable_pose_name_));
// }

 rover_msgs::msg::JointCommandArray SimpleRoverLocomotion::compute_joint_commands(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "X_linear: %f.", msg->linear.x);
  RCLCPP_DEBUG(this->get_logger(), "Y_linear: %f.", msg->linear.y);
  RCLCPP_DEBUG(this->get_logger(), "Z_linear: %f.", msg->linear.z);
  RCLCPP_DEBUG(this->get_logger(), "X_angular: %f.", msg->angular.x);
  RCLCPP_DEBUG(this->get_logger(), "Y_angular: %f.", msg->angular.y);
  RCLCPP_DEBUG(this->get_logger(), "Z_angular: %f.", msg->angular.z);

  // Create Joint Command Message
  rover_msgs::msg::JointCommand joint_commmad_msg;
  // Create JointCommandArray Msg
  rover_msgs::msg::JointCommandArray joint_command_array_msg;

  rover_msgs::msg::JointCommandArray driving_command_array_msg;

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  double x_dot = msg->linear.x;
  double y_dot = msg->linear.y;
  double theta_dot = msg->angular.z;

  // Check if steering is limited and sets y-velocity to 0 if that's the case.
  if (!fully_steerable_) {
    y_dot = 0.0;
  }

  // Check if the rover shall be stopped.
  bool stop_rover = false;

  if (abs(x_dot) == 0.0 &&
      abs(y_dot) == 0.0 &&
      abs(theta_dot) == 0.0)
  {
    stop_rover = true;
  }


  // Compute steering position and driving velocity for each leg
  for (auto leg : rover_->legs_) {

    double alpha;         // [rad] Angle of wheel steering center to origin
    double l;             // [m] Radial Distance from Wheel Steering center to Origin
    double beta_offset;   // [rad] Static offset to calculate steering angle from computed angle
    double beta;          // [rad] Computed angle derived from 2D kinematic model
    double beta_steer;    // [rad] Steering Angle to send to platform driver
    double r;             // [m] Wheel Radius
    double phi_dot;       // [rad/s] Wheel velocity

    double beta_current;  // [rad] Current steering Angle

    double lower_position_limit;
    double upper_position_limit;

    if (leg->is_steerable()) {
      beta_current = leg->steering_motor->current_state->position;
      lower_position_limit = leg->steering_motor->joint->limits->lower;
      upper_position_limit = leg->steering_motor->joint->limits->upper;    
    }
    else {
      beta_current = 0;
      lower_position_limit = 0;
      upper_position_limit = 0;
    }

    // Compute position of wheel center in respect to the center of rotation.
    double offset_str_position_x = leg->driving_motor->global_pose.position.x - centre_of_rotation_x_;
    double offset_str_position_y = leg->driving_motor->global_pose.position.y - centre_of_rotation_y_;

    bool flip_velocity = false;

    // RCLCPP_INFO(this->get_logger(), "Current Steering for %s: %f [rad]", leg->steering_motor->joint->name.c_str(), beta_current);

    if (stop_rover) {
      phi_dot = 0.0;
    }
    else {

      alpha = atan2(offset_str_position_y, offset_str_position_x);
      l = sqrt(pow(offset_str_position_x, 2.0) + pow(offset_str_position_y, 2.0));
      r = leg->wheel_diameter / 2;

      beta_offset = M_PI / 2 - alpha;

      // Compute steering angle from no sliding constraint.
      beta =
        atan2(
        -(sin(alpha) * y_dot + cos(alpha) * x_dot),
        (l * theta_dot + cos(alpha) * y_dot - sin(alpha) * x_dot));

      // Check if leg is steerable
      if (leg->is_steerable()) {
        // Shift steering angle to correct orientation depending on the wheel.
        beta_steer = beta - beta_offset;
        RCLCPP_DEBUG(this->get_logger(),"beta_steer        : %f", beta_steer*180/M_PI);


        // Limit steering angle to +-360
        beta_steer = fmod(beta_steer, 2 * M_PI);
        RCLCPP_DEBUG(this->get_logger(),"beta_steer 360    : %f", beta_steer*180/M_PI);

        // Limit steering angle to +-180
        if (abs(beta_steer) >= M_PI) {
          beta_steer = beta_steer - copysign(2 * M_PI, beta_steer);
        }
        RCLCPP_DEBUG(this->get_logger(),"beta_steer 180    : %f", beta_steer*180/M_PI);

        // Check if Steering angle is within limits and adjust it accordingly
        if (beta_steer < lower_position_limit) {
          beta_steer += M_PI;
          flip_velocity = !flip_velocity;
        }
        else if (beta_steer > upper_position_limit) {
          beta_steer -= M_PI;
          flip_velocity = !flip_velocity;
        }

        // Check if there are multiple ways to arrange wheels
        if ( (beta_steer - M_PI > lower_position_limit) ||
             (beta_steer + M_PI < upper_position_limit))
        {
          // Set steering angle so it's the closest to the current steering angle
          double beta_1 = beta_steer;                                 // Option one is the computed steering angle
          double beta_2 = beta_steer - copysign(M_PI, beta_steer);    // Option two is the computed steering angle flipped 180 deg over the 0 degree point so it stays within the position limit

          double beta_1_diff = abs(beta_1 - beta_current);
          double beta_2_diff = abs(beta_2 - beta_current);

          if (beta_2_diff < beta_1_diff) {
            beta_steer = beta_2;
            flip_velocity = !flip_velocity;
          }
        }

        // Check if wheel is not yet close to the target position
        if (abs(beta_steer - beta_current) > steering_margin_) {
            steering_in_progress_ = true;
        }
      }

      // No need to compute the driving velocity if steering is in progress
      if (steering_in_progress_) {
        phi_dot = 0.0;
      }
      else {
        // Compute Driving Speed
        phi_dot =
          (sin(alpha + beta) * x_dot - cos(alpha + beta) * y_dot - l * cos(beta) * theta_dot) / r;

        if (flip_velocity) {
          phi_dot = -phi_dot;
        }
      }
    }

    // Only compute steering message if leg is steerable
    if (leg->is_steerable()) {
      // Fill Steering Message
      joint_commmad_msg.header.stamp = clock->now();
      joint_commmad_msg.name = leg->steering_motor->joint->name;

      if (stop_rover) {
        joint_commmad_msg.mode = ("VELOCITY");
        joint_commmad_msg.value = 0;
      }
      else {
        joint_commmad_msg.mode = ("POSITION");
        joint_commmad_msg.value = beta_steer;
      }

      joint_command_array_msg.joint_command_array.push_back(joint_commmad_msg);
    }

    // Fill Driving Message
    joint_commmad_msg.header.stamp = clock->now();
    joint_commmad_msg.name = leg->driving_motor->joint->name;
    joint_commmad_msg.mode = ("VELOCITY");
    if (stop_rover) {
      joint_commmad_msg.value = 0;
    }
    else {
      joint_commmad_msg.value = phi_dot;
    }

    driving_command_array_msg.joint_command_array.push_back(joint_commmad_msg);

  }

  // The driving command must be set to zero if ANY steering position has not been reached yet.
  if (steering_in_progress_) {
    for (auto & driving_command : driving_command_array_msg.joint_command_array) {
      driving_command.value = 0.0;
    }
  }

  // Add driving to other joint commands
  joint_command_array_msg.joint_command_array.insert(
    joint_command_array_msg.joint_command_array.end(),
    driving_command_array_msg.joint_command_array.begin(),
    driving_command_array_msg.joint_command_array.end());

  joint_command_array_msg.header.stamp = clock->now();

  steering_in_progress_ = false;

  // Gazebo does not like to receive the steering and driving commands in two different messages.
  // Publish Message
  return joint_command_array_msg;
}

}

int main(int argc, char * argv[])
{
  rclcpp::NodeOptions options;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<locomotion_mode::SimpleRoverLocomotion>(options, "simple_rover_locomotion_node"));
  rclcpp::shutdown();
  return 0;
}
