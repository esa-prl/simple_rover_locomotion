#include "simple_rover_locomotion/locomotion_mode.hpp"

LocomotionMode::LocomotionMode(rclcpp::NodeOptions options)
: Node("locomotion_mode_node",
  options.allow_undeclared_parameters(true).
      automatically_declare_parameters_from_overrides(true)),
  model_(new urdf::Model()),
  joints_(),
  links_(),
  current_joint_state_()
{
  // Load Parameters
  load_params();

  // Load URDF
  load_robot_model();

  // Create Services
  activate_service_ = this->create_service<simple_rover_locomotion::srv::Activate>("activate", std::bind(&LocomotionMode::activate, this, std::placeholders::_1, std::placeholders::_2));
  changelocomotionmode_service_ = this->create_service<simple_rover_locomotion::srv::ChangeLocomotionMode>("change_locomotion_mode", std::bind(&LocomotionMode::change_locomotion_mode, this, std::placeholders::_1, std::placeholders::_2));

  // Create Publishers
  joints_publisher_ = this->create_publisher<simple_rover_locomotion::msg::JointCommandArray>("rover_joint_cmds", 10);
  
  // Create Subscriptions
  joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, std::bind(&LocomotionMode::joint_state_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "LocomotionMode initialized");
}

// Function to be called from the derived class while it is being initialized.
// Creates a subscriber using the (now by derived class overwritten) callback function
void LocomotionMode::initialize_subscribers()
{
  rover_velocities_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
  "rover_motion_cmd", 10, std::bind(&LocomotionMode::rover_velocities_callback, this, std::placeholders::_1));

}

void LocomotionMode::activate(const simple_rover_locomotion::srv::Activate::Request::SharedPtr request,
         std::shared_ptr<simple_rover_locomotion::srv::Activate::Response>      response)
{
    if (request->goal_state) {
        response->new_state = true;
    }
    else {response->new_state = false;}

    RCLCPP_INFO(this->get_logger(), "Goal State %d", request->goal_state);
    RCLCPP_INFO(this->get_logger(), "New State %d", response->new_state);
}

void LocomotionMode::change_locomotion_mode(const simple_rover_locomotion::srv::ChangeLocomotionMode::Request::SharedPtr request,
         std::shared_ptr<simple_rover_locomotion::srv::ChangeLocomotionMode::Response>      response)
{
    // if (request->locomotion_mode == '2D_KINEMATICS' || request->locomotion_mode == 'WHEELWALKING') {
    //     response->response = 'LOCOMOTION CHANGED';
    // }
    // else {response->response = 'INVALID LOCOMOTION MODE REQUESTED';}

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response: %s", response->response.c_str());
    response->response = "SUCCSEFULLY SET " + request->locomotion_mode + " MODE!";
    RCLCPP_INFO(this->get_logger(), "Desired Locomotion Mode %s", request->locomotion_mode.c_str());
}

// Dummy Callback function in case the derived class forgets to create a custom callback function
void LocomotionMode::rover_velocities_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  RCLCPP_INFO(this->get_logger(), "X_linear: %f.", msg->linear.x);
  RCLCPP_INFO(this->get_logger(), "Y_linear: %f.", msg->linear.y);
  RCLCPP_INFO(this->get_logger(), "Z_linear: %f.", msg->linear.z);
  RCLCPP_INFO(this->get_logger(), "X_angular: %f.", msg->angular.x);
  RCLCPP_INFO(this->get_logger(), "Y_angular: %f.", msg->angular.y);

  RCLCPP_WARN(this->get_logger(), "Rover Velocities Callback was not overridden!");
}

// Load Parameters
void LocomotionMode::load_params()
{
  // Look in /demos/demo_nodes_cpp/src/parameters/set_and_get_parameters.cpp for implementation examples

  // TODO: Tried delaring this a member variable, but didn't get it running. Check later if there are better tutorials for param loading. 2019-12-12
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);

  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  model_path_ = parameters_client->get_parameters({"urdf_model_path"})[0].value_to_string();
}

// Load Robot Model (URDF or XACRO)
void LocomotionMode::load_robot_model()
{
    if (!model_->initFile(model_path_)){
      RCLCPP_ERROR(this->get_logger(), "URDF file [%s] not found. Make sure the path is specified in the launch file.", model_path_.c_str());
    }
    else RCLCPP_INFO(this->get_logger(), "Successfully parsed urdf file.");
    
    // Get Links
    model_->getLinks(links_);

    // Printout all Joints and Links

    // RCLCPP_INFO(this->get_logger(), "LINKS:");
    // for (size_t i = 0; i < links_.size(); i++) {
    //   RCLCPP_INFO(this->get_logger(), "\t %s", links_[i]->name.c_str());
    // }
    // RCLCPP_INFO(this->get_logger(), "JOINTS:");
    // Get Joints
    for (size_t i = 0; i < links_.size(); i++) {
      if (links_[i]->child_joints.size() != 0) {
        for (std::shared_ptr<urdf::Joint> child_joint : links_[i]->child_joints) {
          joints_.push_back(child_joint);
          // RCLCPP_INFO(this->get_logger(), "\t %s", child_joint->name.c_str());
        }     
      }   
    }
        
    // Get Driving LINK
    std::string driving_name = ("DRV");

    std::vector<std::shared_ptr<urdf::Link>> driving_links;

    for (size_t i = 0; i < links_.size(); i++) {
      RCLCPP_INFO(this->get_logger(), "%s", links_[i]->name.c_str());
      if (links_[i]->name.find(driving_name) != std::string::npos) {
        driving_links.push_back(links_[i]);
      }
    }


    for (std::shared_ptr<urdf::Link> driving_link : driving_links)
    {
      std::shared_ptr<urdf::Joint> driving_joint = driving_link->parent_joint;

      // Derive 2D Position
      std::vector<double> position = get_parent_joint_position(driving_link);
      
      // // Derive Steering Limits
      double effort = driving_joint->limits->effort;
      // // etc.

      // Check if driving wheel is steerable
      // Derive Steering Joint
      // if (std::shared_ptr<urdf::Joint> steering_joint = get_steering_joint(driving_link)) {
      if (std::shared_ptr<urdf::Joint> steering_joint = get_joint_in_leg(driving_link, ("STR"))) {
        std::cout << "Steering Joint found: ";
        // Compute steering angle and wheel speeds
        std::cout << steering_joint->name << std::endl;

      }

    }

}

// TODO: Pass value instaed of shared_ptr
std::vector<double> LocomotionMode::get_parent_joint_position(std::shared_ptr<urdf::Link> &link) {

  // Copy link so we don't overwrite the original one
  std::shared_ptr<urdf::Link> tmp_link = std::make_shared<urdf::Link>(*link);

  urdf::Pose child_pose;

  while (tmp_link->parent_joint) {
    urdf::Pose parent_pose = tmp_link->parent_joint->parent_to_joint_origin_transform;

    child_pose = transpose_pose(parent_pose, child_pose);

    tmp_link = tmp_link->getParent();
  }
  std::vector<double> position{child_pose.position.x, child_pose.position.y, child_pose.position.z};

  return position;
}

urdf::Pose LocomotionMode::transpose_pose(urdf::Pose parent, urdf::Pose child)
{
  // Based on convention from Hendriks Summary
  // TODO Transform orientation
  urdf::Pose new_child;

  double &e0 = parent.rotation.w;
  double &e1 = parent.rotation.x;
  double &e2 = parent.rotation.y;
  double &e3 = parent.rotation.z;

  double c11 = pow(e0, 2) + pow(e1, 2) - pow(e2, 2) - pow(e3, 2);
  double c12 = 2 * e1 * e2 - 2 * e0 * e3;
  double c13 = 2 * e0 * e2 + 2 * e1 * e3;
  double c21 = 2 * e0 * e3 + 2 * e1 * e2;
  double c22 = pow(e0, 2) - pow(e1, 2) + pow(e2, 2) - pow(e3, 2);
  double c23 = 2 * e2 * e3 - 2 * e0 * e1;
  double c31 = 2 * e1 * e3 - 2 * e0 * e2;
  double c32 = 2 * e0 * e1 + 2 * e2 * e3;
  double c33 = pow(e0, 2) - pow(e1, 2) - pow(e2, 2) + pow(e3, 2);

  double &c14 = parent.position.x;
  double &c24 = parent.position.y;
  double &c34 = parent.position.z;

  new_child.position.x = c11 * child.position.x + c12 * child.position.y + c13 * child.position.z + c14 * 1;
  new_child.position.y = c21 * child.position.x + c22 * child.position.y + c23 * child.position.z + c24 * 1;
  new_child.position.z = c31 * child.position.x + c32 * child.position.y + c33 * child.position.z + c34 * 1;

  return new_child;
}

std::shared_ptr<urdf::Joint> LocomotionMode::get_joint_in_leg(std::shared_ptr<urdf::Link> &link, std::string name) {  
  
  std::shared_ptr<urdf::Link> tmp_link = std::make_shared<urdf::Link>(*link);

  while (tmp_link->parent_joint) {
    if (tmp_link->parent_joint->name.find(name) != std::string::npos) return tmp_link->parent_joint;
    tmp_link = tmp_link->getParent();
  }
}


std::shared_ptr<urdf::Joint> LocomotionMode::get_steering_joint(std::shared_ptr<urdf::Link> &link) {  
  return get_joint_in_leg(link, ("STR"));
}

bool LocomotionMode::is_steerable(std::shared_ptr<urdf::Link> &link) {
  std::cout << "CHECKING IF WHEEL IS STEERABLE: " << std::endl;
  
  std::shared_ptr<urdf::Link> tmp_link = std::make_shared<urdf::Link>(*link);
  
  std::string steering_name = ("STR");

  while (tmp_link->parent_joint) {
    if (tmp_link->parent_joint->name.find(steering_name) != std::string::npos) return true;
    tmp_link = tmp_link->getParent();
  }
  std::cout << "NOT STEERABLE!" << std::endl;

  return false;
}

void LocomotionMode::derive_leg(std::shared_ptr<urdf::Link> &link, std::vector<std::shared_ptr<urdf::Joint>> leg_motors) {
  std::shared_ptr<urdf::Link> tmp_link = std::make_shared<urdf::Link>(*link);

  std::cout << "LEG: " << std::endl;
  
  while (tmp_link->parent_joint) {
    std::cout << "\tLink Name: " << tmp_link->name << std::endl;

    leg_motors.push_back(tmp_link->parent_joint);
    tmp_link = tmp_link->getParent();
  }

}


// Save the joint states into the class
void LocomotionMode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Not ideal since it overrides all previous saved joint states even if they didn't change.
  current_joint_state_ = *msg;
}

// std::vector<double> LocomotionMode::get_joint_