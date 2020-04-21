#include "simple_rover_locomotion/simple_rover_locomotion.hpp"

SimpleRoverLocomotion::SimpleRoverLocomotion(rclcpp::NodeOptions options, std::string node_name) :
LocomotionMode(options, node_name),
// Assuming all rovers are limited except if they show to be unlimited.
fully_steerable_(false),
steering_margin_(20*3.1514/180),
steering_in_progress_(false)
{
  // Create Subscription and callback to derived class method
  if(this->enabled_){
    this->enable_subscribers();
  }

  RCLCPP_INFO(this->get_logger(), "SimpleRoverLocomotion started.");

  if (!check_steering_limitations()) {
    RCLCPP_ERROR(this->get_logger(), "%s not able to work, since the robot model has an invalid steering configuration.", node_name.c_str());
  }
}


bool SimpleRoverLocomotion::check_steering_limitations() {
  std::vector<std::shared_ptr<LocomotionMode::Leg>> non_steerable_legs;

  // Finds the non_steerable legs and saves them.
  for (auto leg : legs_) {
    if (leg->steering_motor->joint->type != urdf::Joint::REVOLUTE &&
          leg->steering_motor->joint->type != urdf::Joint::CONTINUOUS) {
      non_steerable_legs.push_back(leg);
    }
  }

  // Checks how many legs are non steerable and sets the centre of rotation accortingly
  if (non_steerable_legs.size() == 0)
  {
    RCLCPP_WARN(this->get_logger(), "ALL WHEELS ARE STEERABLE");
    centre_of_rotation_x = 0;
    centre_of_rotation_y = 0;
    // Overwrite limited limited steerability
    fully_steerable_ = true;
  }
  else if (non_steerable_legs.size() == 1)
  {
    centre_of_rotation_x = non_steerable_legs[0]->steering_motor->global_pose.position.x;
    // Can be any value. 0 makes the most sense.
    centre_of_rotation_y = 0;
  }
  else if (non_steerable_legs.size() == 2) {
    RCLCPP_WARN(this->get_logger(), "NUMBER OF NON STEERABLE LEGS: %u", non_steerable_legs.size()); 
    // In case there are multiple fixed wheels their axes must align
    // TODO: Check in which direction the wheels point in their 0 position and apply that to the input limitation etc.
    // Check if their axes align with a margin of 1 cm
    if (abs(non_steerable_legs[0]->steering_motor->global_pose.position.x
      - non_steerable_legs[1]->steering_motor->global_pose.position.x) > 0.01) {
      RCLCPP_ERROR(this->get_logger(), "Non steerable wheels don't align.");
      return false;
    }
    // Since they align set the centre of rotation between them.
    centre_of_rotation_x = (non_steerable_legs[0]->steering_motor->global_pose.position.x
                          + non_steerable_legs[1]->steering_motor->global_pose.position.x)/2;
    // This should be 0 in most cases
    centre_of_rotation_y = (non_steerable_legs[0]->steering_motor->global_pose.position.y
                          + non_steerable_legs[1]->steering_motor->global_pose.position.y)/2;
  }
  else {
    RCLCPP_ERROR(this->get_logger(), "MORE THAN TWO NON STEERABLE LEGS");
    return false;
  }

  return true;
}

// See the example bellow how to add a custom transition
// bool SimpleRoverLocomotion::enable(){
//   // Testing of custom transition
//   RCLCPP_INFO(this->get_logger(), "ENABLING SimpleRoverLocomotion using a custom transition");
//   // Transition via a hard coded pose
//   return transition_to_robot_pose("straight");
//   // Transition via the pose specified in the config file.
//   return transition_to_robot_pose(enable_pose_name_));
// }

void SimpleRoverLocomotion::rover_velocities_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "X_linear: %f.", msg->linear.x);
  RCLCPP_DEBUG(this->get_logger(), "Y_linear: %f.", msg->linear.y);
  RCLCPP_DEBUG(this->get_logger(), "Z_linear: %f.", msg->linear.z);
  RCLCPP_DEBUG(this->get_logger(), "X_angular: %f.", msg->angular.x);
  RCLCPP_DEBUG(this->get_logger(), "Y_angular: %f.", msg->angular.y);
  RCLCPP_DEBUG(this->get_logger(), "Z_angular: %f.", msg->angular.z);

  // It's important to check if the message was already initialized
  // TODO: Check if Joint State is older than a certain age and if send a warning/refuse execution accordingly

  // TODO: Add header
  // Create JointCommandArray Msg
  rover_msgs::msg::JointCommandArray joint_command_array_msg;

  rover_msgs::msg::JointCommandArray driving_command_array_msg;

   // Create Steering Joint Message
  rover_msgs::msg::JointCommand steering_msg;

  // Create Driving Joint Message
  rover_msgs::msg::JointCommand driving_msg;

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);


  // TODO: Switch this to pointer?
  double x_dot = msg->linear.x;
  double y_dot = msg->linear.y;
  double theta_dot = msg->angular.z;

  // Checks if steering is limited and sets y-velocity to 0 if that's the case.
  if (!fully_steerable_) {
    y_dot = 0.0;
  }

  // Checks if the rover shall be stopped.
  bool stop_rover = false;

  if (abs(x_dot) == 0.0 &&
      abs(y_dot) == 0.0 &&
      abs(theta_dot) == 0.0)
  {
    stop_rover = true;
  }


  // set the wheel steering
  for (std::shared_ptr<LocomotionMode::Leg> leg : legs_) {

    double alpha;         // [rad] Angle of wheel steering center to origin
    double l;             // [m] Radial Distance from Wheel Steering center to Origin
    double beta_offset;   // [rad] Static offset to calculate steering angle from computed angle
    double beta;          // [rad] Computed angle derived from 2D kinematic model
    double beta_steer;    // [rad] Steering Angle to send to platform driver
    double r;             // [m] Wheel Radius
    double phi_dot;       // [rad/s] Wheel velocity

    double lower_position_limit = leg->steering_motor->joint->limits->lower;  
    double upper_position_limit = leg->steering_motor->joint->limits->upper;

    bool flip_velocity = false;

    int adjustment_count = 0;

    double beta_current = leg->steering_motor->joint_state.position[0];  // Current steering Angle

    // RCLCPP_INFO(this->get_logger(), "Current Steering for %s: %f [rad]", leg->steering_motor->joint->name.c_str(), beta_current);

    if (stop_rover) {
      beta_steer = beta_current;
      phi_dot = 0.0;
    }
    else {

      // Compute offset position in case the steering capabilities are limited.
      double offset_str_position_x = leg->steering_motor->global_pose.position.x - centre_of_rotation_x;
      double offset_str_position_y = leg->steering_motor->global_pose.position.y - centre_of_rotation_y;

      alpha = atan2(offset_str_position_y, offset_str_position_x);
      l = sqrt( pow(offset_str_position_x, 2.0) + pow(offset_str_position_x, 2.0));

      // TODO: Could implment this into main LocomotionMode class and add a field wheel diameter. Also make sure the wheel link is a cylinder.
      if (leg->driving_motor->link->collision->geometry->type == urdf::Geometry::CYLINDER) {
        std::shared_ptr<urdf::Cylinder> cyl = std::static_pointer_cast<urdf::Cylinder>(leg->driving_motor->link->collision->geometry);
        r = cyl->radius;
      }
      else
      {
        r = 0.05;
        RCLCPP_WARN(this->get_logger(), "Wheel Link: %s collision geometry should be a cylinder! Wheel radius set to %f [m]", leg->driving_motor->link->name, r);
      }

      beta_offset = M_PI/2 - alpha;

      // Compute steering angle from no sliding constraint.
      beta = atan2(- (sin(alpha) * y_dot + cos(alpha) * x_dot), (l * theta_dot + cos(alpha) * y_dot - sin(alpha) * x_dot));      
    
      // Shift steering angle to correct orientation depending on the wheel.
      beta_steer = beta - beta_offset;
      // printf("beta_steer        : %f\n",beta_steer*180/M_PI);


      // Limit steering angle to +-360
      beta_steer = fmod(beta_steer, 2*M_PI);
      // printf("beta_steer 360    : %f\n",beta_steer*180/M_PI);

      // Limit steering angle to +-180
      if (abs(beta_steer) >= M_PI) {
        beta_steer = beta_steer - copysign(M_PI, beta_steer);
        flip_velocity = !flip_velocity;
        adjustment_count++;
      }
      // printf("beta_steer 180    : %f\n",beta_steer*180/M_PI);

      // Check if Steering angle is within limits and adjust it accordingly
      if (beta_steer <= lower_position_limit)
      {
        beta_steer = beta_steer + M_PI;
        flip_velocity = !flip_velocity;
        adjustment_count++;
      }

      if (beta_steer >= upper_position_limit)
      {
        beta_steer = beta_steer - M_PI;
        flip_velocity = !flip_velocity;
        adjustment_count++;
      }

      // Checks if there are multiple ways to arrange wheels
      if ( (beta_steer < upper_position_limit && beta_steer > lower_position_limit + M_PI) ||
         (beta_steer > lower_position_limit && beta_steer < upper_position_limit - M_PI))
      {
        // Sets steering angle so it's the closest to the current steering angle
        double beta_1 = beta_steer;                                 // Option one is the computed steering angle
        double beta_2 = beta_steer - copysign(M_PI, beta_steer);    // Option two is the computed steering angle flipped 180 deg over the 0 degree point so it stays within the position limit

        double beta_1_diff = abs(beta_1 - beta_current);
        double beta_2_diff = abs(beta_2 - beta_current);

        if (beta_2_diff < beta_1_diff) {
          beta_steer = beta_2;
          adjustment_count++;
          flip_velocity = !flip_velocity;
        }
      }

      // Only computes the driving velocity if the wheel is close enough to the target position
      if (steering_in_progress_ || abs(beta_steer - beta_current) > steering_margin_)
      {
        steering_in_progress_ = true;
        phi_dot = 0.0;
      }
      else
      {
        // Computes Driving Speed
        // TODO: Compute the wheel speeds from the current wheel orientations and not the set wheel orientations.
        phi_dot = (sin(alpha + beta) * x_dot - cos(alpha + beta) * y_dot - l * cos(beta) * theta_dot)/r;

        if (flip_velocity) {
          phi_dot = -phi_dot;
        }
      }
    }

    // Fills Steering Message
    steering_msg.header.stamp = clock->now(); 
    steering_msg.name = leg->steering_motor->joint->name;
    steering_msg.mode = ("POSITION");
    steering_msg.value = beta_steer;

    joint_command_array_msg.joint_command_array.push_back(steering_msg);
  
    // Fills Driving Message
    driving_msg.header.stamp = clock->now(); 
    driving_msg.name = leg->driving_motor->joint->name;
    driving_msg.mode = ("VELOCITY");
    driving_msg.value = phi_dot;

    driving_command_array_msg.joint_command_array.push_back(driving_msg);

  }

  // The driving command must be set to zero if ANY steering position has not been reached yet.
  if (steering_in_progress_) {
    for (unsigned i = 0; i < driving_command_array_msg.joint_command_array.size(); i++) {
      driving_command_array_msg.joint_command_array[i].value = 0.0;
    }
  }

  // Add driving to other joint commands
  joint_command_array_msg.joint_command_array.insert(
    joint_command_array_msg.joint_command_array.end(),
    driving_command_array_msg.joint_command_array.begin(),
    driving_command_array_msg.joint_command_array.end());

  // joint_command_array_msg.header.stamp = clock->now(); 

  // Gazebo does not like to receive the steering and driving commands in two different messages.
  // Publish Message
  joint_command_publisher_->publish(joint_command_array_msg);

  steering_in_progress_ = false;
}


int main(int argc, char * argv[])
{
rclcpp::NodeOptions options;
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<SimpleRoverLocomotion>(options, "simple_rover_locomotion_node"));
rclcpp::shutdown();
return 0;
}