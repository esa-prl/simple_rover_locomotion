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
    // // Get Joints
    // for (size_t i = 0; i < links_.size(); i++) {
    //   if (links_[i]->child_joints.size() != 0) {
    //     for (std::shared_ptr<urdf::Joint> child_joint : links_[i]->child_joints) {
    //       joints_.push_back(child_joint);
    //       RCLCPP_INFO(this->get_logger(), "\t %s", child_joint->name.c_str());
    //     }     
    //   }   
    // }
}


// Save the joint states into the class
void LocomotionMode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Not ideal since it overrides all previous saved joint states even if they didn't change.
  current_joint_state_ = *msg;
}
